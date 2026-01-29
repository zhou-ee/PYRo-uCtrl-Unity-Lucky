/**
 * @file pyro_module_base.h
 * @brief Header file for the PYRO Module Base Class.
 * PYRO 模块基类头文件。
 *
 * This file defines the `pyro::module_base_t` class template, which serves as
 * the foundation for robot modules. It utilizes the Curiously Recurring
 * Template Pattern (CRTP) to provide a type-safe singleton mechanism (via the
 * `instance()` method) and a callback-driven architecture, combining static
 * type resolution with dynamic FSM execution.
 * 本文件定义了 `pyro::module_base_t` 类模板，作为机器人模块的基础。
 * 它利用奇异递归模板模式 (CRTP) 提供了类型安全的单例机制（通过 `instance()` 方法）
 * 和基于回调驱动的架构，结合了静态类型解析与动态状态机执行。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-01-28
 */

#ifndef __PYRO_MODULE_BASE_H__
#define __PYRO_MODULE_BASE_H__

#include "pyro_core_fsm.h"
#include "pyro_mutex.h"
#include "pyro_task.h"

namespace pyro
{

/**
 * @brief Base command structure.
 * 基础命令结构。
 */
struct cmd_base_t
{
    enum class mode_t : uint8_t { ZERO_FORCE, ACTIVE } mode;
    uint32_t timestamp;
    cmd_base_t() : mode(mode_t::ZERO_FORCE), timestamp(0){}
    virtual ~cmd_base_t() = default;
};

/**
 * @brief CRTP Template for Module Base.
 * 模块基类的 CRTP 模板。
 */
template <typename Derived, typename CmdType>
class module_base_t
{
  public:
    static Derived *instance()
    {
        static Derived _instance_obj; //NOLINT
        return &_instance_obj;
    }

    void start();
    void set_command(const CmdType &cmd);
    [[nodiscard]] mutex_t &get_mutex();

  protected:
    explicit module_base_t(
        const char *name = "module_task", uint16_t init_stack = 512,
        uint16_t loop_stack = 256,
        task_base_t::priority_t priority = task_base_t::priority_t::HIGH);

    virtual ~module_base_t() = default;

    /** @brief Callback for initialization. 初始化回调。 */
    virtual void _init() = 0;

    /** @brief Callback for sensor updates. 反馈更新回调。 */
    virtual void _update_feedback() = 0;

    /** @brief Callback for FSM execution. 状态机执行回调。 */
    virtual void _fsm_execute() = 0;

    CmdType _cmd[2];
    uint8_t _read_index{0};

  private:
    class module_task_t final : public task_base_t
    {
      public:
        module_task_t(module_base_t *owner_ptr, const char *name,
                       uint16_t init_stack, uint16_t loop_stack,
                       priority_t priority);
      protected:
        void init() override;
        void run_loop() override;
      private:
        module_base_t *_owner;
    };

    void _update_command();
    void _run_loop_impl();

    module_task_t _task;
    bool _cmd_updated{false};
    mutex_t _mutex;
};

} // namespace pyro

#include "pyro_module_base.tpp"

#endif