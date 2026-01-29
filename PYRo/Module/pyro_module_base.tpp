/**
 * @file pyro_module_base.tpp
 * @brief Implementation of the PYRO Module Base Template.
 * PYRO 模块基类模板实现文件。
 *
 * This file contains the implementation of the `pyro::module_base_t` template
 * class. It handles the low-level details of command double-buffering,
 * thread-safe mutex locking, and the internal task loop execution logic.
 * 本文件包含了 `pyro::module_base_t` 模板类的实现。它处理命令双缓冲、
 * 线程安全互斥锁以及内部任务循环执行逻辑的底层细节。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-01-28
 */

#pragma once

namespace pyro
{

template <typename Derived, typename CmdType>
module_base_t<Derived, CmdType>::module_base_t(const char *name,
                                               uint16_t init_stack,
                                               uint16_t loop_stack,
                                               task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority)
{
}

template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::start()
{
    _task.start();
}

template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::set_command(const CmdType &cmd)
{
    scoped_mutex_t lock(_mutex);
    _cmd[1 - _read_index] = cmd;
    _cmd_updated          = true;
}

/**
 * @brief Updates command from double buffer.
 * 从双缓冲中更新命令。
 */
template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::_update_command()
{
    if (_cmd_updated)
    {
        scoped_mutex_t lock(_mutex);
        _read_index  = 1 - _read_index;
        _cmd_updated = false;
    }
}

template <typename Derived, typename CmdType>
mutex_t &module_base_t<Derived, CmdType>::get_mutex()
{
    return _mutex;
}

/**
 * @brief Core loop invoking virtual callbacks.
 * 调用虚函数回调的核心循环。
 */
template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::_run_loop_impl()
{
    TickType_t xLastWakeTime        = xTaskGetTickCount();
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
module_base_t<Derived, CmdType>::module_task_t::module_task_t(
    module_base_t *owner_ptr, const char *name, const uint16_t init_stack,
    const uint16_t loop_stack, const priority_t priority)
    : task_base_t(name, init_stack, loop_stack, priority), _owner(owner_ptr)
{
}

/**
 * @brief Invokes the initialization function of the module instance.
 * 调用模块实例的初始化函数。
 */
template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::module_task_t::init()
{
    if (_owner)
        _owner->_init();
}

/**
 * @brief Invokes the core loop implementation of the module instance.
 * 调用模块实例的核心循环实现。
 */
template <typename Derived, typename CmdType>
void module_base_t<Derived, CmdType>::module_task_t::run_loop()
{
    if (_owner)
        _owner->_run_loop_impl();
}

} // namespace pyro