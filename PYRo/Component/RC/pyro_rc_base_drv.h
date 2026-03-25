#ifndef __PYRO_RC_BASE_DRV_H__
#define __PYRO_RC_BASE_DRV_H__

#include "pyro_rw_lock.h"
#include "pyro_uart_drv.h"
#include "message_buffer.h"
#include "pyro_virtual_rc.h"
#include "pyro_task.h"

namespace pyro {

class rc_drv_t : public task_base_t {
public:
    inline static uint8_t sequence = 0x80;

    // 全局唯一虚拟控制器，作为所有驱动映射的终点
    inline static virtual_rc_t shared_v_rc;

    static void init_virtual_rc() { shared_v_rc.init_all(); }

    void enable();
    void disable();

    [[nodiscard]] bool check_online() const;
    [[nodiscard]] rw_lock &get_lock() const;

protected:
    // 构造函数接收引用
    rc_drv_t(uart_drv_t& uart, const char* task_name, uint8_t priority_bit, uint16_t frame_len);
    virtual ~rc_drv_t();

    status_t init() override;
    void run_loop() override;

    virtual bool check_packet(const uint8_t *buf, uint16_t len) = 0;
    virtual void unpack(const uint8_t *buf) = 0;

private:
    bool rc_callback(uint8_t *buf, uint16_t len, BaseType_t& xHigherPriorityTaskWoken);

    rw_lock *_lock{};
    MessageBufferHandle_t _rc_msg_buffer{};
    uart_drv_t& _rc_uart; // 使用引用，绝对保证非空
    uint8_t _priority_bit{};
    uint16_t _frame_len{};
    uint8_t* _rx_buf{};
};

} // namespace pyro
#endif