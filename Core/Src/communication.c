#define _USE_MATH_DEFINES
#include "transducer.h"
#include "dma_manager.h"
#include "communication.h"
#include "utiles.h"

// 全局变量
static rx_buffer_t rx_buffer;

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
                        case CMD_PING:
                            Comm_Handle_Ping_Command(rx_buffer.frame.data, rx_buffer.frame.data_length);
                            break;
                        case CMD_GET_STATUS:
                        {
                            int offset = 0;
                            float voltage_VDDA, voltage_3V3, voltage_5V0, temperature;
                            voltage_VDDA = Get_Voltage_VDDA();
                            voltage_3V3 = Get_Voltage_3V3();
                            voltage_5V0 = Get_Voltage_5V0();
                            temperature = Get_Temperature();
                            float loop_freq = System_Loop_Freq;
                            uint32_t calibration_mode =  Get_Calibration_Mode();
                            uint32_t plane_mode = Get_Plane_Mode();

                            uint8_t response_data[37];

                            memcpy(response_data + offset, &voltage_VDDA, sizeof(voltage_VDDA)); offset += sizeof(voltage_VDDA);
                            memcpy(response_data + offset, &voltage_3V3, sizeof(voltage_3V3)); offset += sizeof(voltage_3V3);
                            memcpy(response_data + offset, &voltage_5V0, sizeof(voltage_5V0)); offset += sizeof(voltage_5V0);

                            memcpy(response_data + offset, &temperature, sizeof(temperature)); offset += sizeof(temperature);
                            // updateDMABufferDeltaTime
                            memcpy(response_data + offset, &updateDMABufferDeltaTime, sizeof(updateDMABufferDeltaTime)); offset += sizeof(updateDMABufferDeltaTime);
                            memcpy(response_data + offset, &loop_freq, sizeof(loop_freq)); offset += sizeof(loop_freq);
                            memcpy(response_data + offset, &CurrentStimulation.type, sizeof(CurrentStimulation.type)); offset += sizeof(CurrentStimulation.type);
                            memcpy(response_data + offset, &calibration_mode, sizeof(calibration_mode)); offset += sizeof(calibration_mode);
                            memcpy(response_data + offset, &plane_mode, sizeof(plane_mode)); offset += sizeof(plane_mode);
                            
                            Comm_Send_Response(RSP_RETURN_STATUS, response_data, sizeof(response_data));
                            break;
                        }
                        case CMD_SET_STIMULATION:
                        {
                            if (rx_buffer.frame.data_length >= 20)
                            {
                                Stimulation stimulation;
                                uint8_t *pData = rx_buffer.frame.data;
                                int offset = 0;
                                StimulationType type = (StimulationType)pData[offset]; offset += 1;
                                switch (type)
                                {
                                case PointStimulation:
                                    memcpy(&stimulation.position[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.position[2], &pData[offset], 4); offset += 4;
                                    break;
                                case VibrationStimulation:
                                    memcpy(&stimulation.startPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[2], &pData[offset], 4); offset += 4;
                                    break;
                                case LinearSTM:
                                    memcpy(&stimulation.startPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.startPoint[2], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.endPoint[2], &pData[offset], 4); offset += 4;
                                    break;
                                case CircularSTM:
                                    memcpy(&stimulation.centerPoint[0], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.centerPoint[1], &pData[offset], 4); offset += 4;
                                    memcpy(&stimulation.centerPoint[2], &pData[offset], 4); offset += 4;
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
                                
                                phase_set_mode = 0;

                                Comm_Send_Response(RSP_SACK, NULL, 0);
                                
                                Set_Stimulation(&stimulation);
                            }
                            else
                            {
                                Comm_Send_Response(RSP_ERROR_CODE, NULL, 0);
                            }
                            break;
                        }
                        case CMD_SET_PHASES:
                        {
                            if (rx_buffer.frame.data_length >= 4)
                            {
                                float phases[NumTransducer-1];
                                uint8_t *pData = rx_buffer.frame.data; // float[NumTransducer-1]
                                memcpy(phases, pData, sizeof(phases));
                                Comm_Send_Response(RSP_SACK, NULL, 0);
                                Set_Phases(phases);
                                CurrentStimulation = EmptyStimulation;
                                phase_set_mode = 1;
                                Update_All_DMABuffer(1);
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