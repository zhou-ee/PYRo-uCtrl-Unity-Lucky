#ifndef __PYRO_QUAD_BOOSTER_H__
#define __PYRO_QUAD_BOOSTER_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct quad_booster_cmd_t final : public cmd_base_t
{
    bool fric_on;     // 摩擦轮开启
    bool fire_enable; // 拨弹开启 (true: 持续增加角度拨弹)
    float fric1_mps;  // 第一级摩擦轮目标转速
    float fric2_mps;  // 第二级摩擦轮目标转速

    quad_booster_cmd_t()
        : fric_on(false), fire_enable(false), fric1_mps(0), fric2_mps(0)
    {
    }
};

// =========================================================
// 2. 四轮发射机构类
// =========================================================
class quad_booster_t final
    : public module_base_t<quad_booster_t, quad_booster_cmd_t>
{
    friend class module_base_t<quad_booster_t, quad_booster_cmd_t>;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    quad_booster_t(const quad_booster_t &)            = delete;
    quad_booster_t &operator=(const quad_booster_t &) = delete;

  private:
    quad_booster_t();
    ~quad_booster_t() override = default;

    // --- 接口实现 ---
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 内部辅助 ---
    void _fric_control();
    void _trigger_control();
    void _send_fric_command() const;
    void _send_trigger_command() const;

    // --- 成员变量 ---
    struct motor_ctx_t
    {
        motor_base_t *fric_wheels[4]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *fric_pid[4]{nullptr}; // 摩擦轮只需速度环

        // 拨弹盘：串级 PID
        pid_t *trigger_pos_pid{nullptr}; // 位置环 (外环)
        pid_t *trigger_spd_pid{nullptr}; // 速度环 (内环)
    };

    struct data_ctx_t
    {
        // 反馈
        float current_fric_mps[4]{};
        float current_trig_radps{0};
        float current_trig_torque{0};
        float current_trig_rad{0}; // -PI ~ PI

        // 目标
        float target_fric_mps[4]{};
        float target_trig_rad{0};   // 目标角度
        float target_trig_radps{0}; // 角度环输出，速度环参考


        // 输出
        float out_fric_torque[4]{};
        float out_trig_torque{0};
    };

    struct booster_ctx_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        quad_booster_cmd_t *cmd{};
    };

    booster_ctx_t _ctx;

    // =====================================================
    // 状态定义 (Nested HFSM)
    // =====================================================
    using owner = quad_booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_homing_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _homing_turnback_time{0.0f};
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_homing_t _homing_state;
        state_ready_t _ready_state;
        state_stall_t _stall_state;
    };
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

    static constexpr float FRIC1_RADIUS = 0.03f; // 第一级摩擦轮半径 (m)
    static constexpr float FRIC2_RADIUS = 0.03f; // 第二级摩擦轮半径 (m)
};

} // namespace pyro
#endif