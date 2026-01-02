/**
 * @file pyro_chassis_base.h
 * @brief Robot chassis base with callback-driven architecture.
 * 基于回调驱动架构的机器人底盘基类。
 */

#ifndef __PYRO_CHASSIS_BASE_H__
#define __PYRO_CHASSIS_BASE_H__

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
    float vx, vy, wz;
    cmd_base_t() : mode(mode_t::ZERO_FORCE), timestamp(0),
                   vx(0), vy(0), wz(0) {}
    virtual ~cmd_base_t() = default;
};

/**
 * @brief CRTP Template for Chassis Base.
 * 底盘基类的 CRTP 模板。
 */
template <typename Derived, typename CmdType>
class chassis_base_t
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
    explicit chassis_base_t(
        const char *name = "chassis", uint16_t init_stack = 512,
        uint16_t loop_stack = 256,
        task_base_t::priority_t priority = task_base_t::priority_t::HIGH);

    virtual ~chassis_base_t() = default;

    /** @brief Callback for initialization. 初始化回调。 */
    virtual void _init() = 0;

    /** @brief Callback for sensor updates. 反馈更新回调。 */
    virtual void _update_feedback() = 0;

    /** @brief Callback for FSM execution. 状态机执行回调。 */
    virtual void _fsm_execute() = 0;

    CmdType _cmd[2];
    uint8_t _read_index{0};

  private:
    class chassis_task_t final : public task_base_t
    {
      public:
        chassis_task_t(chassis_base_t *owner_ptr, const char *name,
                       uint16_t init_stack, uint16_t loop_stack,
                       priority_t priority);
      protected:
        void init() override;
        void run_loop() override;
      private:
        chassis_base_t *_owner;
    };

    void _update_command();
    void _run_loop_impl();

    chassis_task_t _task;
    bool _cmd_updated{false};
    mutex_t _mutex;
};

} // namespace pyro

#include "pyro_chassis_base.tpp"

#endif // __PYRO_CHASSIS_BASE_H__