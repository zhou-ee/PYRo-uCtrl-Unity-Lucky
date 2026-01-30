#include "pyro_com_cantx.h"
#include <cstring>

namespace pyro
{

// 必须在 cpp 中定义静态成员变量，否则链接报错
std::vector<can_tx_drv_t::frame_node_t> can_tx_drv_t::_frames;

can_tx_drv_t::can_tx_drv_t()
{
    _frames.clear();
    // 假设 get_instance() 返回单例
    _can_driver = can_hub_t::get_instance()->hub_get_can_obj(can_hub_t::can1);
}

can_tx_drv_t::~can_tx_drv_t() = default;

can_tx_drv_t *can_tx_drv_t::instance()
{
    static can_tx_drv_t instance;
    return &instance;
}

can_tx_drv_t::frame_node_t *can_tx_drv_t::get_frame(const uint32_t id)
{
    for (auto &frame : _frames)
    {
        if (frame.id == id)
        {
            return &frame;
        }
    }

    frame_node_t new_frame{};
    new_frame.id                = id;
    new_frame.current_bit_usage = 0;
    memset(new_frame.buffer, 0, 8);
    _frames.push_back(new_frame);

    return &_frames.back();
}

void can_tx_drv_t::clear(uint32_t id)
{
    frame_node_t *frame = get_frame(id);
    if (frame)
    {
        frame->current_bit_usage = 0;
        memset(frame->buffer, 0, 8);
    }
}

// 核心优化函数
void can_tx_drv_t::add_data_raw(const uint32_t id, const uint8_t bit_len, const void *data)
{
    if (!data || bit_len == 0) return;

    frame_node_t *frame = get_frame(id);

    // 1. 检查溢出
    if (frame->current_bit_usage + bit_len > 64) return;

    // 2. 准备源数据 (加载到 64位 容器中)
    uint64_t data_val = 0;

    // 计算需要拷贝的字节数，避免读取越界
    // 例如 bit_len=1，只拷贝 1 字节；bit_len=12，拷贝 2 字节
    uint8_t copy_size = (bit_len + 7) / 8;
    if (copy_size > 8) copy_size = 8;

    // 这里的 memcpy 依赖当前 CPU 为小端序 (Little Endian)，ARM/x86 均为小端序，符合 CAN 常规用法
    memcpy(&data_val, data, copy_size);

    // 3. 掩码处理：清除超过 bit_len 的高位脏数据
    if (bit_len < 64)
    {
        data_val &= ((1ULL << bit_len) - 1);
    }

    // 4. 位移处理：将数据移动到当前 buffer 的空闲位置
    // 相当于 "buf位移已使用的长度" 的逆向逻辑 -> 把数据位移到已使用长度之后
    data_val <<= frame->current_bit_usage;

    // 5. 合并写入 (Load-Modify-Store)
    // 使用 memcpy 避免直接指针强转导致的内存对齐(Alignment)问题
    uint64_t buffer_val = 0;
    memcpy(&buffer_val, frame->buffer, 8);

    buffer_val |= data_val; // 核心位操作：直接按位或

    memcpy(frame->buffer, &buffer_val, 8);

    // 6. 更新水位线
    frame->current_bit_usage += bit_len;
}

void can_tx_drv_t::send(const uint32_t id)
{
    // 注意：send 必须由 instance 调用才能访问 _can_driver 实例成员
    // 如果 send 也是 static，则无法访问 _can_driver
    if (!instance()->_can_driver) // 修正逻辑：确保获取单例的 driver
        return;

    for (auto &frame : _frames)
    {
        if (frame.id == id)
        {
            instance()->_can_driver->send_msg(frame.id, frame.buffer);
            return;
        }
    }
}

} // namespace pyro