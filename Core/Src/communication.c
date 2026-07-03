#define _USE_MATH_DEFINES
#include "transducer.h"
#include "dma_manager.h"
#include "communication.h"
#include "stim_types.h"
#include "utiles.h"

// 全局变量
static rx_buffer_t rx_buffer;

/* ---- Command queue: producer = USB RX ISR, consumer = main-loop Comm_Tick ----
 * Single-producer/single-consumer ring of Comm_Command slots. head is the
 * next slot to be filled by the ISR; tail is the next slot to be drained by
 * the main loop. Capacity is one less than the array size to disambiguate
 * full vs. empty. cmd_overflow_cnt counts frames dropped due to a full queue.
 *
 * Index reads/writes on uint8_t are atomic on Cortex-M7, and there is exactly
 * one writer (ISR) and one reader (main loop) for each index, so no further
 * locking is required for SPSC queues. */
#define COMM_CMD_QUEUE_SIZE 4
static Comm_Command  cmd_queue[COMM_CMD_QUEUE_SIZE];
static volatile uint8_t cmd_head = 0;   /* ISR writes here */
static volatile uint8_t cmd_tail = 0;   /* main loop reads here */
static volatile uint16_t cmd_overflow_cnt = 0;

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void)
{
    Comm_Reset_Rx_State();
}

/**
 * @brief 重置接收状态
 */
void Comm_Reset_Rx_State(void)
{
    rx_buffer.state = RX_STATE_WAIT_HEADER1;
    rx_buffer.data_index = 0;
    rx_buffer.calculated_checksum = 0;
    memset(&rx_buffer.frame, 0, sizeof(comm_frame_t));
}

void Comm_Check_Rx_Timeout(void)
{
    if (rx_buffer.state != RX_STATE_WAIT_HEADER1) {
        if (HAL_GetTick() - rx_buffer.last_byte_tick > RX_FRAME_TIMEOUT_MS) {
            Comm_Reset_Rx_State();
        }
    }
}

/**
 * @brief 计算校验和
 * @param cmd_type 命令类型
 * @param data_length 数据长度
 * @param data 数据指针
 * @return 校验和
 */
uint8_t Comm_Calculate_Checksum(uint8_t cmd_type, uint8_t data_length, const uint8_t* data)
{
    uint16_t sum = cmd_type + data_length;

    for (uint8_t i = 0; i < data_length; i++) {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief 发送响应帧
 * @param cmd_type 响应类型
 * @param data 数据指针
 * @param data_length 数据长度
 */
void Comm_Send_Response(uint8_t cmd_type, const uint8_t* data, uint8_t data_length)
{
    uint8_t tx_buffer[COMM_MAX_FRAME]; // 最大帧长度 = 7 + 255
    uint16_t index = 0;

    // 帧头
    tx_buffer[index++] = FRAME_HEADER_1;
    tx_buffer[index++] = FRAME_HEADER_2;

    // 命令类型
    tx_buffer[index++] = cmd_type;

    // 数据长度
    tx_buffer[index++] = data_length;

    // 数据载荷
    if (data_length > 0 && data != NULL) {
        memcpy(&tx_buffer[index], data, data_length);
        index += data_length;
    }

    // 校验和
    tx_buffer[index++] = Comm_Calculate_Checksum(cmd_type, data_length, data);

    // 帧尾
    tx_buffer[index++] = FRAME_TAIL_1;
    tx_buffer[index++] = FRAME_TAIL_2;

    /* CDC_Transmit_FS returns USBD_BUSY when the host is not draining the
     * endpoint fast enough; retry briefly to avoid silently dropping
     * responses. Safe to block here: Comm_Send_Response now runs from the
     * main loop (Comm_Tick), not from the USB ISR. */
    uint8_t retry = 0;
    while (CDC_Transmit_FS(tx_buffer, index) == USBD_BUSY && retry < 2) {
        HAL_Delay(1);
        retry++;
    }
}

/**
 * @brief 处理Ping命令
 * @param data 接收到的数据
 * @param data_length 数据长度
 */
void Comm_Handle_Ping_Command(const uint8_t* data, uint8_t data_length)
{
    // 将接收到的随机数原样返回
    Comm_Send_Response(RSP_PING_ACK, data, data_length);
}

/**
 * @brief 处理接收到的数据
 * @param data 接收到的数据缓冲区
 * @param length 数据长度
 */
void Comm_Process_Received_Data(uint8_t* data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        rx_buffer.last_byte_tick = HAL_GetTick();

        switch (rx_buffer.state) {
            case RX_STATE_WAIT_HEADER1:
                if (byte == FRAME_HEADER_1) {
                    rx_buffer.frame.header[0] = byte;
                    rx_buffer.state = RX_STATE_WAIT_HEADER2;
                }
                break;

            case RX_STATE_WAIT_HEADER2:
                if (byte == FRAME_HEADER_2) {
                    rx_buffer.frame.header[1] = byte;
                    rx_buffer.state = RX_STATE_WAIT_CMD_TYPE;
                } else {
                    Comm_Reset_Rx_State();
                }
                break;

            case RX_STATE_WAIT_CMD_TYPE:
                rx_buffer.frame.cmd_type = byte;
                rx_buffer.calculated_checksum = byte;
                rx_buffer.state = RX_STATE_WAIT_DATA_LENGTH;
                break;

            case RX_STATE_WAIT_DATA_LENGTH:
                rx_buffer.frame.data_length = byte;
                rx_buffer.calculated_checksum += byte;
                rx_buffer.data_index = 0;

                if (rx_buffer.frame.data_length == 0) {
                    rx_buffer.state = RX_STATE_WAIT_CHECKSUM;
                } else {
                    rx_buffer.state = RX_STATE_WAIT_DATA;
                }
                break;

            case RX_STATE_WAIT_DATA:
                rx_buffer.frame.data[rx_buffer.data_index] = byte;
                rx_buffer.calculated_checksum += byte;
                rx_buffer.data_index++;

                if (rx_buffer.data_index >= rx_buffer.frame.data_length) {
                    rx_buffer.state = RX_STATE_WAIT_CHECKSUM;
                }
                break;

            case RX_STATE_WAIT_CHECKSUM:
                rx_buffer.frame.checksum = byte;

                // 验证校验和
                if ((rx_buffer.calculated_checksum & 0xFF) == byte) {
                    rx_buffer.state = RX_STATE_WAIT_TAIL1;
                } else {
                    // 校验和错误，重置状态
                    Comm_Reset_Rx_State();
                }
                break;

            case RX_STATE_WAIT_TAIL1:
                if (byte == FRAME_TAIL_1) {
                    rx_buffer.frame.tail[0] = byte;
                    rx_buffer.state = RX_STATE_WAIT_TAIL2;
                } else {
                    Comm_Reset_Rx_State();
                }
                break;

            case RX_STATE_WAIT_TAIL2:
                if (byte == FRAME_TAIL_2) {
                    rx_buffer.frame.tail[1] = byte;
                    rx_buffer.state = RX_STATE_FRAME_COMPLETE;

                    /* Frame is complete: enqueue a command for the main loop
                     * instead of executing it here (we are in the USB RX ISR
                     * context). Heavy work (Update_Full_Waveform_Buffer, EEPROM
                     * writes, SSD1306_Flush) must NOT run in ISR. */
                    uint8_t next_head = (uint8_t)((cmd_head + 1U) % COMM_CMD_QUEUE_SIZE);
                    if (next_head != cmd_tail) {
                        Comm_Command *slot = &cmd_queue[cmd_head];
                        slot->cmd_type    = rx_buffer.frame.cmd_type;
                        slot->data_length = rx_buffer.frame.data_length;
                        if (rx_buffer.frame.data_length > 0)
                            memcpy(slot->data, rx_buffer.frame.data, rx_buffer.frame.data_length);
                        cmd_head = next_head;
                    } else {
                        /* Queue full: drop this frame and signal overflow. */
                        cmd_overflow_cnt++;
                        Comm_Send_Response(RSP_NACK, NULL, 0);
                    }
                }

                // 处理完成后重置状态
                Comm_Reset_Rx_State();
                break;

            case RX_STATE_FRAME_COMPLETE:
                // 这个状态不应该到达，重置状态
                Comm_Reset_Rx_State();
                break;
        }
    }
}

/**
 * @brief 执行一条已解析完成的命令 (在主循环上下文中调用)。
 *        所有 Set_Stimulation / Set_Transducers / EEPROM / SSD1306 等重操作
 *        均在此处执行, 避免 ISR 内重计算与共享状态竞态。
 */
static void Comm_Execute_Command(const Comm_Command *cmd)
{
    switch (cmd->cmd_type) {
        case CMD_ENABLE_DISABLE:
        {
            if (cmd->data_length >= 1)
            {
                uint8_t enable = cmd->data[0];
                if (enable)
                {
                    Stimulation_Enable();
                }
                else
                {
                    Stimulation_Disable();
                }
                Comm_Send_Response(RSP_ACK, NULL, 0);
            }
            else
            {
                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
            }
            break;
        }
        case CMD_PING:
            Comm_Handle_Ping_Command((uint8_t *)cmd->data, cmd->data_length);
            break;
        case COM_GET_CONFIG:
        {
            device_config config;
            memset(&config, 0, sizeof(config));

            // 设备序列号
            char* serial_number = Get_Device_Serial_Number();
            strncpy(config.serial_number, serial_number, sizeof(config.serial_number) - 1);
            config.serial_number[sizeof(config.serial_number) - 1] = '\0';

            config.version = VERSION;
            config.array_type = ARRAY_TYPE_CONCENTRIC_RINGS;
            config.array_size = NUM_RINGS;
            config.num_transducer = NUM_REAL_TRANSDUCER;
            config.transducer_size = TRANSDUCER_SIZE;
            config.transducer_space = TRANSDUCER_SPACING;

            Comm_Send_Response(RSP_RETURN_CONFIG, (uint8_t*)&config, sizeof(config));
            break;
        }
        case CMD_GET_STATUS:
        {
            device_status status;

            status.voltage_VDDA = Get_Voltage_VDDA();
            status.voltage_3V3 = 0.0f;
            status.voltage_5V0 = 0.0f;
            status.temperature = Get_Temperature();
            status.updateDMABufferDeltaTime = updateDMABufferDeltaTime;
            status.loop_freq = System_Loop_Freq;
            status.stimulation_type = Stim_Get_Type_Id(&CurrentStimulation);
            status.calibration_mode = Get_Calibration_Mode();
            status.phase_set_mode = Get_Phase_Set_Mode();

            Comm_Send_Response(RSP_RETURN_STATUS, (uint8_t*)&status, sizeof(status));
            break;
        }
        case CMD_SET_STIMULATION:
        {
            if (cmd->data_length >= 3)
            {
                uint8_t type_id = cmd->data[0];
                const StimTypeDescriptor *td = Stim_Get_Type_By_Id(type_id);

                if (td && td->deserialize)
                {
                    Stimulation s;
                    memset(&s, 0, sizeof(s));
                    s.type_id   = type_id;
                    s.type_desc = td;
                    strncpy(s.name, td->name, sizeof(s.name) - 1);

                    uint8_t ok = td->deserialize(&s,
                        &cmd->data[1],
                        cmd->data_length - 1);

                    if (ok)
                    {
                        Set_Stimulation(&s);
                        phase_set_mode = 0;
                        Comm_Send_Response(RSP_SACK, NULL, 0);
                    }
                    else
                    {
                        Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                    }
                }
                else
                {
                    Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                }
            }
            else
            {
                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
            }
            break;
        }
        case CMD_SET_TRANSDUCERS:
        {
            if (cmd->data_length >= (NUM_REAL_TRANSDUCER) * SERIAL_TRANSDUCER_BYTES)
            {
                uint8_t *pData = (uint8_t *)cmd->data;

                CurrentStimulation = EmptyStimulation;
                phase_set_mode = 1;

                Set_Transducers(pData);
                Comm_Send_Response(RSP_SACK, NULL, 0);
            }
            else
            {
                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
            }
            break;
        }
        case CMD_SET_DEMO:
        {
            if (cmd->data_length >= 2)
            {
                uint8_t name_len = cmd->data[0];
                const char *name = (const char *)&cmd->data[1];

                const StimDemoDescriptor *demo = Stim_Get_Demo_By_Name(name, name_len);
                if (demo)
                {
                    demo_mode = Stim_Get_Demo_Index(demo);
                    Set_Stimulation_From_Demo(demo);
                    phase_set_mode = 0;

                    Comm_Send_Response(RSP_DEMO_ACK, (uint8_t *)demo->name, (uint8_t)strlen(demo->name));
                }
                else
                {
                    Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                }
            }
            else
            {
                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
            }
            break;
        }
        case CMD_GET_TRANSDUCER_INFO:
        {
            if (cmd->data_length >= 2)
            {
                uint8_t start_index = cmd->data[0];
                uint8_t count = cmd->data[1];

                if (count > MAX_TRANSDUCER_INFO_PER_REQUEST) count = MAX_TRANSDUCER_INFO_PER_REQUEST;

                if (start_index >= NUM_REAL_TRANSDUCER)
                {
                    count = 0;
                }
                else if (start_index + count > NUM_REAL_TRANSDUCER)
                {
                    count = NUM_REAL_TRANSDUCER - start_index;
                }

                uint8_t resp_data[2 + MAX_TRANSDUCER_INFO_PER_REQUEST * TRANSDUCER_POSITION_BYTES];
                uint8_t resp_len = 0;

                resp_data[resp_len++] = start_index;
                resp_data[resp_len++] = count;

                for (uint8_t i = 0; i < count; i++)
                {
                    uint8_t idx = start_index + i;
                    memcpy(&resp_data[resp_len], TransducerArray[idx].position3D, TRANSDUCER_POSITION_BYTES);
                    resp_len += TRANSDUCER_POSITION_BYTES;
                }

                Comm_Send_Response(RSP_TRANSDUCER_INFO, resp_data, resp_len);
            }
            else
            {
                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
            }
            break;
        }
        default:
            // 未知命令，返回NACK
            Comm_Send_Response(RSP_NACK, NULL, 0);
            break;
    }
}

/**
 * @brief 主循环调用: 出队并执行至多一条命令。
 *        ISR 仅入队, 此处完成所有重操作, 避免 ISR 内重计算与共享状态竞态。
 */
void Comm_Tick(void)
{
    if (cmd_head != cmd_tail) {
        Comm_Command *slot = &cmd_queue[cmd_tail];
        Comm_Execute_Command(slot);
        cmd_tail = (uint8_t)((cmd_tail + 1U) % COMM_CMD_QUEUE_SIZE);
    }
}
