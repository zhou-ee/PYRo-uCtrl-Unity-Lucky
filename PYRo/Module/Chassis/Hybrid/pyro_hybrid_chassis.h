#ifndef __PYRO_HYBRID_CHASSIS_H__
#define __PYRO_HYBRID_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_kin_hybrid.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct hybrid_cmd_t : cmd_base_t
{
    hybrid_kin_t::drive_mode_t drive_mode;
    uint8_t leg_contract_mode;
    uint8_t jump_mode;
    float vx,vy,wz,wy;

    hybrid_cmd_t()
        : drive_mode(hybrid_kin_t::drive_mode_t::CRUISING),
          leg_contract_mode(0), jump_mode(0), vx(0), vy(0), wz(0), wy(0)
    {
    }
};

// =========================================================
// 2. 混合底盘类
// =========================================================
class hybrid_chassis_t final
    : public module_base_t<hybrid_chassis_t, hybrid_cmd_t>
{

    // 前向声明
    friend class chassis_base_t;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct hybrid_context_t;

  public:
    hybrid_chassis_t(const hybrid_chassis_t &)            = delete;
    hybrid_chassis_t &operator=(const hybrid_chassis_t &) = delete;

  private:
    hybrid_chassis_t();
    ~hybrid_chassis_t() override = default;

    // --- 基类接口 ---
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _kinematics_solve();
    static void _chassis_control(hybrid_context_t *ctx);
    static void _send_motor_command(hybrid_context_t *ctx);




    hybrid_kin_t *_kinematics{nullptr};

    // 电机句柄
    struct motor_ctx_t
    {
        motor_base_t *mecanum[4]{nullptr};
        // motor_base_t *track[2]{nullptr};
        motor_base_t *leg[2]{nullptr};
    };

    // 算法对象
    struct pid_ctx_t
    {
        pid_t *mecanum_pid[4]{nullptr};
        pid_t *track_pid[2]{nullptr};
        pid_t *leg_pos_pid[2]{nullptr};
        pid_t *leg_spd_pid[2]{nullptr};
        pid_t *balance_pid{nullptr};
    };

    // 运行时数据
    struct data_ctx_t
    {
        float current_wheel_rpm[4]{};
        float current_track_rpm[2]{};
        float current_leg_rad[2]{};
        float current_leg_radps[2]{};

        float target_wheel_rpm[4]{};
        float target_track_rpm[2]{};
        float target_leg_rad[2]{};
        float target_leg_radps[2]{};

        // 输出
        float out_mecanum_torque[4]{};
        float out_track_torque[2]{};
        float out_leg_torque[2]{};
    };

    struct hardware_ctx_t
    {
        powermeter_drv_t *power_meter{nullptr};
    };

    struct power_ctx_t
    {
        powermeter_data *data{nullptr};
    };



    struct hybrid_context_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        hardware_ctx_t hardware;
        power_ctx_t power;
        data_ctx_t data;
        hybrid_cmd_t *cmd;
    };
    struct debug_ctx_t
    {
        float debug_leg_torque[2]{};
    };
    // 总 Context
    hybrid_context_t _ctx;
    debug_ctx_t debug_data;


    // =====================================================
    // 状态定义 (HFSM)
    // 注意：不再需要 sibling/next 指针，直接查 _ctx.states
    // =====================================================

    // 1. 被动状态
    using owner = hybrid_chassis_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 2. 主动状态 (FSM)
    struct fsm_active_t : public fsm_t<owner>
    {
        // 子状态定义
        struct state_cruising_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_climbing_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_jumping_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };


        // FSM Hooks
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_cruising_t _cruising_state;
        state_climbing_t _climbing_state;
        state_jumping_t _jumping_state;
    };

    // 状态实例
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 静态纯逻辑内核
    // =====================================================
    // static void _pure_update_feedback(motor_ctx_t &hw, data_ctx_t &data);
    // static void _pure_calc_kinematics(const algo_ctx_t &algo, data_ctx_t
    // &data); static void _pure_calc_mecanum_pid(algo_ctx_t &algo, data_ctx_t
    // &data); static void _pure_calc_track_pid(algo_ctx_t &algo, data_ctx_t
    // &data); static void _pure_calc_leg_pid(algo_ctx_t &algo, data_ctx_t
    // &data); static void _pure_reset_pids(algo_ctx_t &algo); static void
    // _pure_hw_write(motor_ctx_t &hw, const data_ctx_t &data);
    //
    // // 辅助
    // static float _mps_to_rpm(float mps, float radius);
    // static float _radps_to_rpm(float radps);
    static constexpr float MEC_RADIUS      = 0.076f;
    static constexpr float TRACK_RADIUS    = 0.035f;
    static constexpr float LEG_RETRACT_POS = 0.6f;
    static constexpr float LEG_EXTEND_POS  = 1.60f;
};

} // namespace pyro

#endif
