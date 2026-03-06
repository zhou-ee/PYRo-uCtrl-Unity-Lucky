/**
 * @file pyro_vl53_drv.cpp
 * @brief Implementation of VL53-100 Laser Ranging Sensor Driver
 */

#include "pyro_vl53_drv.h"
#include "pyro_core_dma_heap.h"
#include <cstring>
#include <cstdlib>

namespace pyro
{

vl53_drv_t& vl53_drv_t::get_instance()
{
    static vl53_drv_t instance;
    return instance;
}

vl53_drv_t::vl53_drv_t()
{
    // 申请一块安全的 DMA 内存池用于发送指令，避免栈内存被非阻塞 DMA 销毁
    _dma_tx_buf = static_cast<uint8_t*>(pvPortDmaMalloc(8));
    if (_dma_tx_buf) {
        memset(_dma_tx_buf, 0, 8);
    }
}

vl53_drv_t::~vl53_drv_t()
{
    if (_uart) {
        _uart->remove_rx_event_callback(_owner_id);
    }
    if (_dma_tx_buf) {
        vPortFree(_dma_tx_buf);
        _dma_tx_buf = nullptr;
    }
}

status_t vl53_drv_t::init(uart_drv_t* uart, uint8_t modbus_id, uint32_t owner_id)
{
    // if (!uart || !_dma_tx_buf) return PYRO_ERROR;
    //
    // _uart = uart;
    // _modbus_id = modbus_id;
    // _owner_id = owner_id;
    //
    // // 1. 注册统一接收回调
    // _uart->add_rx_event_callback(
    //     [this](uint8_t* p, uint16_t size, BaseType_t& awoken) -> bool {
    //         return this->rx_callback(p, size, awoken);
    //     },
    //     _owner_id
    // );
    //
    // // ==========================================
    // // 配置阶段：传感器出厂默认为 115200。如果目标波特率为 921600，需动态切换。
    // // 为了保证兼容性，这里我们先将 MCU 串口临时设为 115200 发送波特率切换指令
    // // ==========================================
    //
    // // 临时重置 MCU UART 为 115200 (传感器出厂默认)
    // // _uart->reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE);
    // // HAL_Delay(20);
    // //
    // // // 向传感器下发配置的波特率指令 (例如 921600)
    // write_register(VL53_REG_BAUD_RATE, vl53_config::BAUD_RATE, 100);
    // HAL_Delay(50); // 等待传感器应用新波特率
    // //
    // // // 将 MCU UART 正式重置为用户配置的目标波特率 (例如 921600)
    // // _uart->reset(vl53_config::REAL_BAUD, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE);
    // // HAL_Delay(20);
    // //
    // // // 开启 DMA 接收
    // // if (_uart->enable_rx_dma() != PYRO_OK) {
    // //     return PYRO_ERROR;
    // // }
    //
    // // ==========================================
    // // 正常下发其他配置 (测量模式 与 系统模式)
    // // ==========================================
    //
    // // 写入测量模式 (例如：一般模式)
    // write_register(VL53_REG_MEASURE_MODE, vl53_config::MEASURE_MODE, 50);
    // HAL_Delay(20);
    //
    // // 写入系统工作模式 (正常自动回传 或 Modbus 模式)
    // write_register(VL53_REG_SYSTEM_MODE, vl53_config::SYSTEM_MODE, 50);
    // HAL_Delay(20);

    return PYRO_OK;
}

status_t vl53_drv_t::request_distance() const
{
    // 如果是自动回传模式，不需要主动请求
    if (vl53_config::SYSTEM_MODE == VL53_SYS_MODE_AUTO_NORMAL) {
        return PYRO_OK;
    }

    if (!_uart || !_dma_tx_buf) return PYRO_ERROR;

    _dma_tx_buf[0] = _modbus_id;
    _dma_tx_buf[1] = VL53_FUNC_READ_REG;
    _dma_tx_buf[2] = 0x00;
    _dma_tx_buf[3] = VL53_REG_DISTANCE;
    _dma_tx_buf[4] = 0x00;
    _dma_tx_buf[5] = 0x01;
    uint16_t crc = calculate_crc16(_dma_tx_buf, 6);
    _dma_tx_buf[6] = crc & 0xFF;
    _dma_tx_buf[7] = (crc >> 8) & 0xFF;

    // 非阻塞 DMA 发送
    return _uart->write(_dma_tx_buf, 8);
}

status_t vl53_drv_t::restore_factory() const
{
    return write_register(VL53_REG_SYS_RESET, VL53_CMD_RESTORE_FACTORY, 100);
}

status_t vl53_drv_t::start_calibration() const
{
    return write_register(VL53_REG_CALIBRATION, VL53_CMD_START_CALIBRATION, 100);
}

bool vl53_drv_t::is_data_fresh()
{
    if (_fresh_flag) {
        _fresh_flag = false;
        return true;
    }
    return false;
}

// 辅助函数：写单个寄存器 (阻塞式，用于初始化和配置)
status_t vl53_drv_t::write_register(uint8_t reg_addr, uint8_t value, uint32_t timeout) const
{
    if (!_uart || !_dma_tx_buf) return PYRO_ERROR;

    _dma_tx_buf[0] = _modbus_id;
    _dma_tx_buf[1] = VL53_FUNC_WRITE_REG; // 0x06
    _dma_tx_buf[2] = 0x00;
    _dma_tx_buf[3] = reg_addr;
    _dma_tx_buf[4] = 0x00;
    _dma_tx_buf[5] = value;

    uint16_t crc = calculate_crc16(_dma_tx_buf, 6);
    _dma_tx_buf[6] = crc & 0xFF;
    _dma_tx_buf[7] = (crc >> 8) & 0xFF;

    return _uart->write(_dma_tx_buf, 8, timeout);
}

// 统一的中断回调入口
bool vl53_drv_t::rx_callback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken)
{
    if (vl53_config::SYSTEM_MODE == VL53_SYS_MODE_AUTO_NORMAL) {
        return parse_ascii_stream(p, size);
    } else {
        return parse_modbus_frame(p, size);
    }
}

// 自动回传模式解析 (ASCII 字符串)
// 格式如: "State:0, Range Valid\r\n d: 73 mm\r\n"
bool vl53_drv_t::parse_ascii_stream(uint8_t* p, uint16_t size)
{
    bool consumed = false;

    // 为安全起见，防止越界，我们用手写状态机简单搜索 'd' ':' ' '
    for (uint16_t i = 0; i < size - 4; ++i) {
        // 解析状态
        if (p[i] == 'S' && p[i+1] == 't' && p[i+2] == 'a' && p[i+3] == 't' && p[i+4] == 'e') {
            if (i + 6 < size) {
                _sensor_state = p[i+6] - '0'; // 提取 '0'~'5'
            }
        }

        // 解析距离值
        if (p[i] == 'd' && p[i+1] == ':') {
            uint16_t start_idx = i + 2;

            // 跳过空格
            while (start_idx < size && p[start_idx] == ' ') {
                start_idx++;
            }

            // 提取数字
            uint16_t temp_dist = 0;
            bool found_num = false;
            while (start_idx < size && p[start_idx] >= '0' && p[start_idx] <= '9') {
                temp_dist = temp_dist * 10 + (p[start_idx] - '0');
                start_idx++;
                found_num = true;
            }

            if (found_num) {
                _distance_mm = temp_dist;
                _fresh_flag = true;
                consumed = true;
                i = start_idx; // 跳过已解析的部分
            }
        }
    }
    return consumed;
}

// Modbus 轮询模式解析 (Hex)
bool vl53_drv_t::parse_modbus_frame(uint8_t* p, uint16_t size)
{
    if (size < 7) return false;

    bool consumed = false;
    for (uint16_t i = 0; i <= size - 7; ++i)
    {
        if (p[i] == _modbus_id && p[i+1] == VL53_FUNC_READ_REG)
        {
            uint8_t data_len = p[i+2];
            uint16_t frame_len = 3 + data_len + 2;

            if (i + frame_len <= size)
            {
                uint16_t calc_crc = calculate_crc16(&p[i], frame_len - 2);
                uint8_t crcl = p[i + frame_len - 2];
                uint8_t crch = p[i + frame_len - 1];

                if (crcl == (calc_crc & 0xFF) && crch == ((calc_crc >> 8) & 0xFF))
                {
                    if (data_len >= 2) {
                        _distance_mm = (p[i+3] << 8) | p[i+4];
                        _sensor_state = VL53_STATE_VALID; // Modbus 返回值不包含状态，默认赋 Valid
                        _fresh_flag = true;
                    }
                    consumed = true;
                    i += (frame_len - 1);
                }
            }
        }
    }
    return consumed;
}

uint16_t vl53_drv_t::calculate_crc16(const uint8_t* buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

} // namespace pyro