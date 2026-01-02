/**
 * @file pyro_chassis_base.tpp
 * @brief Implementation of chassis callback logic.
 * 底盘回调逻辑的实现。
 */
#pragma once

namespace pyro
{

template <typename Derived, typename CmdType>
chassis_base_t<Derived, CmdType>::chassis_base_t(
    const char *name, uint16_t init_stack, uint16_t loop_stack,
    task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority) {}

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::start() { _task.start(); }

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::set_command(const CmdType &cmd)
{
    scoped_mutex_t lock(_mutex);
    _cmd[1 - _read_index] = cmd;
    _cmd_updated = true;
}

/**
 * @brief Updates command from double buffer.
 * 从双缓冲中更新命令。
 */
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::_update_command()
{
    if (_cmd_updated)
    {
        scoped_mutex_t lock(_mutex);
        _read_index = 1 - _read_index;
        _cmd_updated = false;
    }
}

template <typename Derived, typename CmdType>
mutex_t &chassis_base_t<Derived, CmdType>::get_mutex() { return _mutex; }

/**
 * @brief Core loop invoking virtual callbacks.
 * 调用虚函数回调的核心循环。
 */
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::_run_loop_impl()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true)
    {
        _update_command();
        _update_feedback();
        _fsm_execute();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* Internal Task Proxy Implementations --------------------------------------*/

template <typename Derived, typename CmdType>
chassis_base_t<Derived, CmdType>::chassis_task_t::chassis_task_t(
    chassis_base_t *owner_ptr, const char *name, const uint16_t init_stack,
    const uint16_t loop_stack, const priority_t priority)
    : task_base_t(name, init_stack, loop_stack, priority), _owner(owner_ptr) {}

/** @brief Proxy to owner initialization. 代理至所有者的初始化。 */
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::chassis_task_t::init()
{
    if (_owner) _owner->_init();
}

/** @brief Proxy to owner loop. 代理至所有者的循环逻辑。 */
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::chassis_task_t::run_loop()
{
    if (_owner) _owner->_run_loop_impl();
}

} // namespace pyro