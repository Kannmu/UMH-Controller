#define _USE_MATH_DEFINES
#include "transducer.h"
#include "dma_manager.h"
#include "communication.h"
#include "utiles.h"

// 全局变量
static rx_buffer_t rx_buffer;

// 中断到主循环的延迟处理缓冲
static uint8_t rx_pending_data[512];
static volatile uint32_t rx_pending_len = 0;
static volatile int rx_pending_flag = 0;

/**
 * @brief 从USB中断中调用，仅拷贝数据不处理
 * @param data 接收到的数据缓冲区
 * @param len 数据长度
 */
void Comm_Notify_Rx(uint8_t* data, uint32_t len)
{
    if (len > sizeof(rx_pending_data)) len = sizeof(rx_pending_data);
    memcpy(rx_pending_data, data, len);
    rx_pending_len = len;
    rx_pending_flag = 1;
}

/**
 * @brief 主循环中调用，检查并处理待处理数据
 */
void Comm_Process_Pending_Rx(void)
{
    if (rx_pending_flag)
    {
        rx_pending_flag = 0;
        Comm_Process_Received_Data(rx_pending_data, rx_pending_len);
    }
}

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
    uint8_t tx_buffer[260]; // 最大帧长度
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
                            
                            // 设备序列号 (24 hex chars + null)
                            char* serial_number = Get_Device_Serial_Number();
                            strncpy(config.serial_number, serial_number, sizeof(config.serial_number) - 1);
                            config.serial_number[sizeof(config.serial_number) - 1] = '\0';
                            
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

                            Read_Device_Sensors(&status.voltage_VDDA, &status.voltage_3V3,
                                                &status.voltage_5V0, &status.temperature);
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
                            uint8_t *pData = rx_buffer.frame.data;
                            uint32_t data_len = rx_buffer.frame.data_length;

                            if (data_len < 1)
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                                break;
                            }

                            StimulationType type = (StimulationType)pData[0];
                            uint32_t min_len;
                            switch (type)
                            {
                                case Point:
                                    min_len = 1 + 12 + 8; // type + 3 floats pos + 2 floats strength/freq
                                    break;
                                case Discrete:
                                    min_len = 1 + 12 + 12 + 4 + 4 + 8; // type + pos + normal + radius + segments + s/f
                                    break;
                                case Linear:
                                    min_len = 1 + 12 + 12 + 4 + 8; // type + start + end + segments + s/f
                                    break;
                                case Circular:
                                case Square:
                                case STM_Triangle:
                                case Zigzag:
                                case ArchimedeanSpiralInward:
                                case ArchimedeanSpiralOutward:
                                    min_len = 1 + 12 + 12 + 4 + 8; // type + pos + normal + radius + s/f
                                    break;
                                default:
                                    Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                                    break;
                            }

                            if (data_len < min_len)
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                                break;
                            }

                            {
                                Stimulation stimulation;
                                memset(&stimulation, 0, sizeof(Stimulation));
                                stimulation.segments = 1;
                                stimulation.normalVector[2] = 1.0f;
                                int offset = 0;
                                stimulation.type = type; offset += 1;
                                switch (type)
                                {
                                case Point:
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
                                case Circular:
                                case Square:
                                case STM_Triangle:
                                case Zigzag:
                                case ArchimedeanSpiralInward:
                                case ArchimedeanSpiralOutward:
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
                                
                                Set_Stimulation(&stimulation);
                                
                                phase_set_mode = 0;

                                Comm_Send_Response(RSP_SACK, NULL, 0);

                            }
                            break;
                        }
                        case CMD_SET_TRANSDUCERS:
                        {
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
                            if (rx_buffer.frame.data_length >= 1)
                            {
                                uint8_t index = rx_buffer.frame.data[0];
                                int num_demos = Get_Num_Demo_Stimulations();

                                if (index < num_demos)
                                {
                                    demo_mode = index;
                                    Set_Stimulation(DemoStimulations[index]);
                                    phase_set_mode = 0;

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