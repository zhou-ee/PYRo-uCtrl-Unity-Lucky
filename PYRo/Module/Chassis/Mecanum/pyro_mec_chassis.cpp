#include "pyro_mec_chassis.h"

namespace pyro
{

// =========================================================
// 静态辅助函数
// =========================================================
static float _mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1e-4f)
        return 0.0f;
    // v = w * r -> w = v / r
    // RPM = w * 60 / 2pi
    return (mps / radius) * 9.5492966f;
}

static float _radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

// =========================================================
// 构造与初始化
// =========================================================

mec_chassis_t::mec_chassis_t() : module_base_t("mec_chassis")
{
    _ctx = {};
}

void mec_chassis_t::_init()
{
    // 1. 初始化运动学解算器
    _kinematics = new mecanum_kin_t(WHEELBASE, TRACK_WIDTH);

    // 2. 初始化电机 (假设使用 M3508，CAN1, ID 1-4)
    _ctx.motor.wheels[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1); // FL
    _ctx.motor.wheels[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can1); // FR
    _ctx.motor.wheels[2] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can1); // BL
    _ctx.motor.wheels[3] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can1); // BR

    // 3. 初始化 PID (参数需根据实际负载调整，这里使用经验值)
    // KPI, KII, KDI, IntegMax, OutMax, ...
    for (auto &pid : _ctx.pid.wheel_pid)
    {
        pid = new pid_t(0.35f, 0.0008f, 0.0f, 1.0f, 20.0f, 20, 10, 4);
    }
}

// =========================================================
// 核心循环回调
// =========================================================

void mec_chassis_t::_update_feedback()
{
    // 1. 更新电机底层数据
    for (auto *motor : _ctx.motor.wheels)
    {
        motor->update_feedback();
    }

    // 2. 转换数据供上层使用 (RPM)
    // 注意：M3508 减速比处理通常在驱动层或此处处理，假设 update_feedback 得到的是减速后轴的 rad/s
    // 需要乘以减速比倒数来获得轮子实际转速
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] = _radps_to_rpm(
            _ctx.motor.wheels[i]->get_current_rotate() * dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
    }
}

void mec_chassis_t::_kinematics_solve()
{
    // 1. 调用运动学解算
    auto wheel_speeds_mps = _kinematics->solve(_ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz);

    // 2. 将 m/s 转换为 RPM
    _ctx.data.target_wheel_rpm[0] = _mps_to_rpm(wheel_speeds_mps.fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] = -_mps_to_rpm(wheel_speeds_mps.fr, WHEEL_RADIUS); // 右侧电机通常需要反转
    _ctx.data.target_wheel_rpm[2] = _mps_to_rpm(wheel_speeds_mps.bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] = -_mps_to_rpm(wheel_speeds_mps.br, WHEEL_RADIUS); // 右侧电机通常需要反转
}

void mec_chassis_t::_chassis_control(mec_context_t *ctx)
{
    // 计算 PID
    for (int i = 0; i < 4; i++)
    {
        ctx->data.out_wheel_torque[i] = ctx->pid.wheel_pid[i]->calculate(
            ctx->data.target_wheel_rpm[i], ctx->data.current_wheel_rpm[i]);
    }
}

void mec_chassis_t::_send_motor_command(mec_context_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        ctx->motor.wheels[i]->send_torque(ctx->data.out_wheel_torque[i]);
    }
}

// =========================================================
// 状态机逻辑
// =========================================================

void mec_chassis_t::_fsm_execute()
{
    // 1. 获取最新命令指针
    _ctx.cmd = &_cmd[_read_index];

    // 2. 模式切换检查
    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    // 3. 执行当前状态
    _main_fsm.execute(this);
}





} // namespace pyro