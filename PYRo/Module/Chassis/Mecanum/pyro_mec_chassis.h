#ifndef __PYRO_MEC_CHASSIS_H__
#define __PYRO_MEC_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_kin_mec.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct mec_cmd_t final : public cmd_base_t
{
    float vx; // x轴速度 m/s
    float vy; // y轴速度 m/s
    float wz; // z轴角速度 rad/s

    mec_cmd_t() : vx(0), vy(0), wz(0)
    {
    }
};

// =========================================================
// 2. 麦轮底盘类
// =========================================================
class mec_chassis_t final : public module_base_t<mec_chassis_t, mec_cmd_t>
{
    // 允许 module_base_t 创建本类的单例对象（访问私有构造函数）
    friend class module_base_t<mec_chassis_t, mec_cmd_t>;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct mec_context_t;

  public:
    /**
     * @brief Public singleton accessor for code that needs an instance.
     * This constructs the singleton inside the derived class scope, so the
     * private constructor is accessible and no access errors occur when
     * other translation units request the chassis singleton.
     */
    mec_chassis_t(const mec_chassis_t &)            = delete;
    mec_chassis_t &operator=(const mec_chassis_t &) = delete;

  private:
    mec_chassis_t();
    ~mec_chassis_t() override = default;

    // --- 基类接口实现 ---
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    void _kinematics_solve();
    static void _chassis_control(mec_context_t *ctx);
    static void _send_motor_command(mec_context_t *ctx);

    // --- 成员变量 ---
    mecanum_kin_t *_kinematics{nullptr};

    // 电机句柄
    struct motor_ctx_t
    {
        motor_base_t *wheels[4]{nullptr}; // FL, FR, BL, BR
    };

    // 算法对象
    struct pid_ctx_t
    {
        pid_t *wheel_pid[4]{nullptr};
    };

    // 运行时数据
    struct data_ctx_t
    {
        float current_wheel_rpm[4]{};
        float target_wheel_rpm[4]{};
        float out_wheel_torque[4]{};
    };

    // 总 Context
    struct mec_context_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        mec_cmd_t *cmd;
    };

    mec_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = mec_chassis_t;

    // 1. 无力状态 (Zero Force)
    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 2. 有力状态 (Active Control)
    struct state_active_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 状态实例
    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 物理参数常量
    // =====================================================
    static constexpr float WHEEL_RADIUS = 0.076f; // m
    static constexpr float WHEELBASE    = 0.375f; // m (前后轴距)
    static constexpr float TRACK_WIDTH  = 0.380f; // m (左右轮距)
};

} // namespace pyro
#endif

