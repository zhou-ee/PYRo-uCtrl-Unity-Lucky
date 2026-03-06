/**
 * @file pyro_vl53_drv.h
 * @brief VL53-100 Laser Ranging Sensor Driver
 */

#ifndef __PYRO_VL53_DRV_H__
#define __PYRO_VL53_DRV_H__

#include "pyro_uart_drv.h"
#include "VL53_reg.h"

namespace pyro
{

/**
 * @brief VL53-100 编译期配置参数 (修改此处以改变传感器行为)
 */
namespace vl53_config
{
    // 系统模式：VL53_SYS_MODE_AUTO_NORMAL (自动回传) 或 VL53_SYS_MODE_MODBUS (被动轮询)
    constexpr uint8_t SYSTEM_MODE   = VL53_SYS_MODE_AUTO_NORMAL;

    // 测量模式：一般模式
    constexpr uint8_t MEASURE_MODE  = VL53_MEAS_MODE_NORMAL;

    // 目标波特率：921600
    constexpr uint8_t BAUD_RATE     = VL53_BAUD_115200;
    constexpr uint32_t REAL_BAUD    = 921600; // 对应底层真实的波特率数值
}

class vl53_drv_t
{
public:
    static vl53_drv_t& get_instance();

    vl53_drv_t(const vl53_drv_t&) = delete;
    vl53_drv_t& operator=(const vl53_drv_t&) = delete;

    /**
     * @brief 初始化传感器，并自动下发编译期配置
     * @param uart 底层 UART 驱动指针
     * @param modbus_id Modbus 地址 (出厂默认 0x50)
     * @param owner_id 回调id
     */
    status_t init(uart_drv_t* uart, uint8_t modbus_id = 0x50, uint32_t owner_id = 0x5353);

    /**
     * @brief 发送 Modbus 距离请求指令 (仅在 SYSTEM_MODE 为 MODBUS 模式时有效)
     */
    status_t request_distance() const;

    /**
     * @brief 恢复出厂设置
     */
    status_t restore_factory() const;

    /**
     * @brief 触发校准
     */
    status_t start_calibration() const;

    /**
     * @brief 获取最新解析出的距离数据
     * @return 距离 (单位: mm)
     */
    [[nodiscard]] uint16_t get_distance() const { return _distance_mm; }

    /**
     * @brief 获取传感器硬件状态 (0: 正常, 1-5: 各种错误)
     */
    [[nodiscard]] uint8_t get_state() const { return _sensor_state; }

    /**
     * @brief 检查数据是否有效更新，读取后自动清除标志位
     */
    bool is_data_fresh();

private:
    vl53_drv_t();
    ~vl53_drv_t();

    bool rx_callback(uint8_t* p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken);

    // 自动回传模式下的 ASCII 字符串解析
    bool parse_ascii_stream(uint8_t* p, uint16_t size);

    // Modbus 模式下的 Hex 数据帧解析
    bool parse_modbus_frame(uint8_t* p, uint16_t size);

    // 辅助发送指令的方法
    status_t write_register(uint8_t reg_addr, uint8_t value, uint32_t timeout = 50) const;
    static uint16_t calculate_crc16(const uint8_t* buffer, uint16_t length);

    uart_drv_t* _uart{nullptr};
    uint8_t     _modbus_id{0x50};
    uint32_t    _owner_id{0};

    uint8_t* _dma_tx_buf{nullptr}; // 统一的 DMA 发送缓冲池

    volatile uint16_t _distance_mm{0};
    volatile uint8_t  _sensor_state{VL53_STATE_NO_UPDATE};
    volatile bool     _fresh_flag{false};
};

} // namespace pyro

#endif // __PYRO_VL53_DRV_H__