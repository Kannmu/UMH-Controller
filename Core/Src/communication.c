#define _USE_MATH_DEFINES
#include "transducer.h"
#include "dma_manager.h"
#include "communication.h"
#include "indexed_linear.h"
#include "utiles.h"
#include "sequence_player.h"

// 全局变量
static rx_buffer_t rx_buffer;

#define COMM_RX_QUEUE_SIZE 4096U
#define COMM_RX_QUEUE_MASK (COMM_RX_QUEUE_SIZE - 1U)

static uint8_t comm_rx_queue[COMM_RX_QUEUE_SIZE];
static volatile uint16_t comm_rx_head;
static volatile uint16_t comm_rx_tail;
static volatile uint32_t comm_rx_dropped_bytes;

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void)
{
    comm_rx_head = 0U;
    comm_rx_tail = 0U;
    comm_rx_dropped_bytes = 0U;
    Comm_Reset_Rx_State();
}

void Comm_Queue_Received_Data(const uint8_t* data, uint32_t length)
{
    uint16_t head = comm_rx_head;

    for (uint32_t i = 0; i < length; i++)
    {
        uint16_t next_head = (uint16_t)((head + 1U) & COMM_RX_QUEUE_MASK);
        if (next_head == comm_rx_tail)
        {
            comm_rx_dropped_bytes += length - i;
            break;
        }

        comm_rx_queue[head] = data[i];
        head = next_head;
    }

    __DMB();
    comm_rx_head = head;
}

uint32_t Comm_Get_Rx_Dropped_Bytes(void)
{
    return comm_rx_dropped_bytes;
}

void Comm_Task(void)
{
    uint16_t tail = comm_rx_tail;
    uint16_t head = comm_rx_head;
    uint32_t remaining_budget = 64U;

    while (tail != head && remaining_budget > 0U)
    {
        uint32_t contiguous_length = (head > tail)
            ? (uint32_t)(head - tail)
            : (COMM_RX_QUEUE_SIZE - (uint32_t)tail);
        if (contiguous_length > 64U)
        {
            contiguous_length = 64U;
        }
        if (contiguous_length > remaining_budget)
        {
            contiguous_length = remaining_budget;
        }

        Comm_Process_Received_Data(&comm_rx_queue[tail], contiguous_length);
        remaining_budget -= contiguous_length;
        tail = (uint16_t)((tail + contiguous_length) & COMM_RX_QUEUE_MASK);
        __DMB();
        comm_rx_tail = tail;
        head = comm_rx_head;
    }
}

static uint8_t Comm_Get_Stimulation_Payload_Length(StimulationType type)
{
    switch (type)
    {
    case Point:
    case TwinTrap:
        return 21U;
    case Discrete:
        return 41U;
    case Linear:
    case Circular:
        return 37U;
    case IndexedLinear:
        return INDEXED_LINEAR_PAYLOAD_LENGTH;
    default:
        return 0U;
    }
}

static int Comm_Is_Stimulation_Finite(const Stimulation *stimulation)
{
    const float *values = stimulation->position;
    for (uint32_t i = 0; i < 3U; i++) if (!isfinite(values[i])) return 0;
    values = stimulation->startPoint;
    for (uint32_t i = 0; i < 3U; i++) if (!isfinite(values[i])) return 0;
    values = stimulation->endPoint;
    for (uint32_t i = 0; i < 3U; i++) if (!isfinite(values[i])) return 0;
    values = stimulation->normalVector;
    for (uint32_t i = 0; i < 3U; i++) if (!isfinite(values[i])) return 0;

    return isfinite(stimulation->radius) &&
           isfinite(stimulation->strength) &&
           isfinite(stimulation->frequency);
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

/**
 * @brief 计算校验和
 * @param cmd_type 命令类型
 * @param data_length 数据长度
 * @param data 数据指针
 * @return 校验和
 */
uint8_t Comm_Calculate_Checksum(uint8_t cmd_type, uint8_t data_length, uint8_t* data)
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
void Comm_Send_Response(uint8_t cmd_type, uint8_t* data, uint8_t data_length)
{
    uint8_t tx_buffer[262]; // 2-byte header + 255-byte payload + 5-byte framing
    uint8_t index = 0;
    
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
    
    // 通过USB CDC发送
    CDC_Transmit_FS(tx_buffer, index);
}

/**
 * @brief 处理Ping命令
 * @param data 接收到的数据
 * @param data_length 数据长度
 */
void Comm_Handle_Ping_Command(uint8_t* data, uint8_t data_length)
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
                    
                    // 处理完整的帧
                    switch (rx_buffer.frame.cmd_type) {
                        case CMD_ENABLE_DISABLE:
                        {
                            Sequence_Abort();
                            if (rx_buffer.frame.data_length >= 1)
                            {
                                uint8_t enable = rx_buffer.frame.data[0];
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
                            Comm_Handle_Ping_Command(rx_buffer.frame.data, rx_buffer.frame.data_length);
                            break;
                        case COM_GET_CONFIG:
                        {
                            device_config config;
                            memset(&config, 0, sizeof(config));
                            
                            // 设备序列号
                            char* serial_number = Get_Device_Serial_Number();
                            memcpy(config.serial_number, serial_number, 12);
                            
                            config.version = VERSION;
                            config.array_type = 0x01; // 0x00: Rect, 0x01: Hex
                            config.array_size = ARRAY_SIZE;
                            config.num_transducer = NUM_TRANSDUCER;
                            config.transducer_size = TRANSDUCER_SIZE;
                            config.transducer_space = TRANSDUCER_SPACING;
                            
                            Comm_Send_Response(RSP_RETURN_CONFIG, (uint8_t*)&config, sizeof(config));
                            break;
                        }
                        case CMD_GET_STATUS:
                        {
                            device_status status;
                            float voltage_vdda;
                            float voltage_3v3;
                            float voltage_5v0;
                            float temperature;

                            Get_Device_Measurements(&voltage_vdda, &voltage_3v3,
                                                    &voltage_5v0, &temperature);
                            status.voltage_VDDA = voltage_vdda;
                            status.voltage_3V3 = voltage_3v3;
                            status.voltage_5V0 = voltage_5v0;
                            status.temperature = temperature;
                            status.updateDMABufferDeltaTime = updateDMABufferDeltaTime;
                            status.loop_freq = System_Loop_Freq;
                            status.stimulation_type = (uint8_t)CurrentStimulation.type;
                            status.calibration_mode = Get_Calibration_Mode();
                            status.phase_set_mode = Get_Phase_Set_Mode();
                            
                            Comm_Send_Response(RSP_RETURN_STATUS, (uint8_t*)&status, sizeof(status));
                            break;
                        }
                        case CMD_SET_STIMULATION:
                        {
                            Sequence_Abort();
                            if (rx_buffer.frame.data_length < 1U)
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                                break;
                            }
                            StimulationType requested_type = (StimulationType)rx_buffer.frame.data[0];
                            uint8_t required_length = Comm_Get_Stimulation_Payload_Length(requested_type);
                            int valid_length = required_length > 0U &&
                                rx_buffer.frame.data_length >= required_length;
                            if (requested_type == IndexedLinear)
                            {
                                valid_length = rx_buffer.frame.data_length ==
                                               INDEXED_LINEAR_PAYLOAD_LENGTH;
                            }
                            if (valid_length)
                            {
                                Stimulation stimulation;
                                memset(&stimulation, 0, sizeof(Stimulation));
                                stimulation.segments = 1; // Default segments
                                stimulation.normalVector[2] = 1.0f; // Default normal vector Z
                                uint8_t *pData = rx_buffer.frame.data;
                                int offset = 0;
                                StimulationType type = (StimulationType)pData[offset]; offset += 1;
                                stimulation.type = type;
                                switch (type)
                                {
                                case Point:
                                case TwinTrap:
                                    memcpy(&stimulation.position[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[2], &pData[offset], 4); offset += 4;
                                    break;
                                case Discrete:
                                    memcpy(&stimulation.position[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[2], &pData[offset], 4); offset += 4;
                                    
                                    memcpy(&stimulation.normalVector[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.normalVector[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.normalVector[2], &pData[offset], 4); offset += 4;
                                    
                                    memcpy(&stimulation.radius, &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.segments, &pData[offset], 4); offset += 4;
                                    break;
                                case Linear:
                                    memcpy(&stimulation.startPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.segments, &pData[offset], 4); offset += 4;
                                    break;
                                case IndexedLinear:
                                    memcpy(&stimulation.startPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[2], &pData[offset], 4); offset += 4;
                                    offset += 2; // sample count and control flags
                                    offset += INDEXED_LINEAR_SAMPLE_COUNT;
                                    break;
                                case Circular:
                                    memcpy(&stimulation.position[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[2], &pData[offset], 4); offset += 4;
                                    
                                    memcpy(&stimulation.normalVector[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.normalVector[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.normalVector[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.radius, &pData[offset], 4); offset += 4;
                                    break;
                                default:
                                    break;
                                }

                                memcpy(&stimulation.strength,    &pData[offset], 4); offset += 4;
                                memcpy(&stimulation.frequency,   &pData[offset], 4); 
                                
                                int valid_stimulation = Comm_Is_Stimulation_Finite(&stimulation);
                                if (type == IndexedLinear)
                                {
                                    valid_stimulation = valid_stimulation &&
                                        Indexed_Linear_Validate_Payload(
                                            pData, rx_buffer.frame.data_length);
                                    if (valid_stimulation)
                                    {
                                        valid_stimulation = Set_Indexed_Linear_Stimulation(
                                            &stimulation,
                                            &pData[INDEXED_LINEAR_ORDER_OFFSET],
                                            pData[INDEXED_LINEAR_SAMPLE_COUNT_OFFSET],
                                            pData[INDEXED_LINEAR_CONTROL_FLAGS_OFFSET]);
                                    }
                                }

                                if (valid_stimulation)
                                {
                                    if (type != IndexedLinear)
                                    {
                                        Set_Stimulation(&stimulation);
                                    }
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
                            break;
                        }
                        case CMD_SET_TRANSDUCERS:
                        {
                            Sequence_Abort();
                            if (rx_buffer.frame.data_length >= (NUM_TRANSDUCER - 1) * 3)
                            {
                                uint8_t *pData = rx_buffer.frame.data; 
                                
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
                            Sequence_Abort();
                            if (rx_buffer.frame.data_length >= 1)
                            {
                                uint8_t index = rx_buffer.frame.data[0];
                                int num_demos = Get_Num_Demo_Stimulations();

                                if (index < num_demos)
                                {
                                    demo_mode = index;
                                    Set_Stimulation(DemoStimulations[index]);

                                    // Send ACK with name
                                    const char *name = DemoStimulations[index]->name;
                                    Comm_Send_Response(RSP_DEMO_ACK, (uint8_t *)name, strlen(name));
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
                            if (rx_buffer.frame.data_length >= 2)
                            {
                                uint8_t start_index = rx_buffer.frame.data[0];
                                uint8_t count = rx_buffer.frame.data[1];
                                
                                // 限制每次请求的最大数量 (255 - 2) / 12 = 21
                                if (count > 21) count = 21;
                                
                                // 检查范围
                                if (start_index >= NUM_TRANSDUCER)
                                {
                                    count = 0;
                                }
                                else if (start_index + count > NUM_TRANSDUCER)
                                {
                                    count = NUM_TRANSDUCER - start_index;
                                }
                                
                                // 构建响应数据
                                // Format: [Start_Index] [Count] [Data...]
                                uint8_t resp_data[2 + 21 * 12]; 
                                uint8_t resp_len = 0;
                                
                                resp_data[resp_len++] = start_index;
                                resp_data[resp_len++] = count;
                                
                                for (uint8_t i = 0; i < count; i++)
                                {
                                    uint8_t idx = start_index + i;
                                    // 复制 X, Y, Z (3 * 4 = 12 bytes)
                                    memcpy(&resp_data[resp_len], TransducerArray[idx].position3D, 12);
                                    resp_len += 12;
                                }
                                
                                Comm_Send_Response(RSP_TRANSDUCER_INFO, resp_data, resp_len);
                            }
                            else
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                            }
                            break;
                        }
                        case SEQUENCE_GET_CAPS:
                        {
                            SequenceCapabilities capabilities;
                            Sequence_Get_Capabilities(&capabilities);
                            Comm_Send_Response(RSP_ACK, (uint8_t *)&capabilities,
                                               (uint8_t)sizeof(capabilities));
                            break;
                        }
                        case SEQUENCE_BEGIN:
                        {
                            if (rx_buffer.frame.data_length == sizeof(SequenceDescriptor))
                            {
                                SequenceDescriptor descriptor;
                                memcpy(&descriptor, rx_buffer.frame.data, sizeof(descriptor));
                                Comm_Send_Response(
                                    Sequence_Begin_Configuration(&descriptor) ? RSP_ACK : RSP_ERROR_CODE,
                                    NULL, 0U);
                            }
                            else
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0U);
                            }
                            break;
                        }
                        case SEQUENCE_UPLOAD:
                        {
                            if (rx_buffer.frame.data_length >= 3U)
                            {
                                uint16_t first_state;
                                uint8_t count = rx_buffer.frame.data[2];
                                memcpy(&first_state, &rx_buffer.frame.data[0], sizeof(first_state));
                                uint32_t expected = 3U + (uint32_t)count * SEQUENCE_OUTPUT_CHANNELS;
                                int valid = count > 0U && count <= SEQUENCE_MAX_UPLOAD_STATES &&
                                    expected == rx_buffer.frame.data_length;
                                if (valid)
                                {
                                    valid = Sequence_Upload_States(
                                        first_state, count, &rx_buffer.frame.data[3]);
                                }
                                Comm_Send_Response(valid ? RSP_ACK : RSP_ERROR_CODE, NULL, 0U);
                            }
                            else
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0U);
                            }
                            break;
                        }
                        case SEQUENCE_COMMIT:
                            Comm_Send_Response(Sequence_Commit() ? RSP_ACK : RSP_ERROR_CODE,
                                               NULL, 0U);
                            break;
                        case SEQUENCE_DATA:
                        {
                            if (rx_buffer.frame.data_length == 4U + SEQUENCE_PACKET_SAMPLES * 2U)
                            {
                                uint32_t sequence;
                                int16_t samples[SEQUENCE_PACKET_SAMPLES];
                                memcpy(&sequence, &rx_buffer.frame.data[0], sizeof(sequence));
                                memcpy(samples, &rx_buffer.frame.data[4], sizeof(samples));
                                Sequence_Push_Data(sequence, samples, SEQUENCE_PACKET_SAMPLES);
                            }
                            else
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0U);
                            }
                            break;
                        }
                        case SEQUENCE_GET_STATUS:
                        {
                            SequenceStatus status;
                            Sequence_Get_Status(&status, Comm_Get_Rx_Dropped_Bytes());
                            Comm_Send_Response(RSP_RETURN_STATUS, (uint8_t *)&status,
                                               (uint8_t)sizeof(status));
                            break;
                        }
                        default:
                            // 未知命令，返回NACK
                            Comm_Send_Response(RSP_NACK, NULL, 0);
                            break;
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
