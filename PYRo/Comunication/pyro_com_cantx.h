#ifndef __PYRO_COM_CANTX_H__
#define __PYRO_COM_CANTX_H__

#include "pyro_can_drv.h"
#include <vector>
#include <cstdint>
#include <cstring> // for memcpy

namespace pyro
{

class can_tx_drv_t
{
public:
    static can_tx_drv_t *instance();

    // 1. 清空指定 ID 的发送缓存
    static void clear(uint32_t id);

    /**
     * @brief 2. [模板版本] 向指定 ID 的缓存追加数据 (推荐使用)
     * 支持直接传入数值变量，自动处理指针
     * * @tparam T 数据类型
     * @param id CAN ID
     * @param bit_len 数据占用位数 (需小于等于 64)
     * @param data 数据值 (如局部变量)
     */
    template <typename T>
    static void add_data(uint32_t id, uint8_t bit_len, T data)
    {
        // 调用底层的 raw 实现，传入地址
        add_data_raw(id, bit_len, &data);
    }

    /**
     * @brief 2.1 [指针版本] 兼容旧接口或数组传输
     */
    static void add_data_ptr(uint32_t id, uint8_t bit_len, const void *data)
    {
        add_data_raw(id, bit_len, data);
    }

    // 3. 发送指定 ID 的当前缓存
    static void send(uint32_t id);

private:
    can_tx_drv_t();
    ~can_tx_drv_t();

    // 内部实际执行函数 (重命名为 raw 以区分模板)
    static void add_data_raw(uint32_t id, uint8_t bit_len, const void *data);

    struct frame_node_t
    {
        uint32_t id;
        uint8_t current_bit_usage;
        uint8_t buffer[8];
    };

    static frame_node_t* get_frame(uint32_t id);

    // 注意：静态成员需要在 cpp 中定义
    static std::vector<frame_node_t> _frames;
    can_drv_t *_can_driver;
};

}

#endif