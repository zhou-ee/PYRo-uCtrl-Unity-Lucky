#include "pyro_rc_base_drv.h"

namespace pyro {

rc_drv_t::rc_drv_t(uart_drv_t& uart, const char* task_name, uint8_t priority_bit, uint16_t frame_len)
    : task_base_t(task_name, 0, 256, priority_t::REALTIME), 
      _rc_uart(uart), // 在初始化列表中绑定引用
      _priority_bit(priority_bit), 
      _frame_len(frame_len) 
{
    sequence = 0x80;
}

status_t rc_drv_t::init() {
    _rc_msg_buffer = xMessageBufferCreate(_frame_len * 6 + 12); 
    if (_rc_msg_buffer == nullptr) return PYRO_ERROR;

    _rx_buf = new uint8_t[_frame_len];
    _lock = new rw_lock;
    return PYRO_OK;
}

void rc_drv_t::enable() {
    // 引用可以直接使用 . 操作符
    _rc_uart.add_rx_event_callback(
        [this](uint8_t *buf, const uint16_t len, BaseType_t& xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

void rc_drv_t::disable() {
    sequence &= ~(1 << _priority_bit);
    _rc_uart.remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
}

bool rc_drv_t::rc_callback(uint8_t *buf, uint16_t len, BaseType_t& xHigherPriorityTaskWoken) {
    if (len == _frame_len && check_packet(buf, len)) {
        if (__builtin_ctz(sequence) >= _priority_bit) { 
            xMessageBufferSendFromISR(_rc_msg_buffer, buf, len, &xHigherPriorityTaskWoken);
            return true;
        }
    }
    return false;
}

void rc_drv_t::run_loop() {
    if (xMessageBufferReceive(_rc_msg_buffer, _rx_buf, _frame_len, portMAX_DELAY) == _frame_len) {
        sequence |= (1 << _priority_bit);
    }

    while (sequence >> _priority_bit & 0x01) {
        size_t bytes = xMessageBufferReceive(_rc_msg_buffer, _rx_buf, _frame_len, pdMS_TO_TICKS(100));
        if (bytes == _frame_len) {
            unpack(_rx_buf); 
        } else if (bytes == 0) {
            sequence &= ~(1 << _priority_bit); 
        }
    }
}

rw_lock &rc_drv_t::get_lock() const { return *_lock; }

bool rc_drv_t::check_online() const { return sequence >> _priority_bit & 0x01; }

rc_drv_t::~rc_drv_t() {
    if (_rc_msg_buffer) {
        vMessageBufferDelete(_rc_msg_buffer);
        _rc_msg_buffer = nullptr;
    }
    delete[] _rx_buf;
    delete _lock;
}

}