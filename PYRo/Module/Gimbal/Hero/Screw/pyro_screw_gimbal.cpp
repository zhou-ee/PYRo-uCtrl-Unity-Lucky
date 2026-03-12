#include "pyro_screw_gimbal.h"
#include "pyro_ins.h"
#include "pyro_com_canrx.h"

namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

screw_gimbal_t::screw_gimbal_t() : module_base_t("screw_gimbal")
{
    _ctx = {};
}

status_t screw_gimbal_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid = _module_deps.pid_deps;
    pyro::can_rx_drv_t::subscribe(can_hub_t::which_can::can1, 0x102);
    return PYRO_OK;
}

// =========================================================
// 核心循环回调
// =========================================================

void screw_gimbal_t::_update_feedback()
{
    // 1. 刷新电机数据
    _ctx.motor.pitch->update_feedback();
    _ctx.motor.yaw->update_feedback();


    // 读取 IMU 数据作为云台姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_yaw_rad,
                                          &_ctx.data.current_pitch_rad,
                                          &_ctx.data.current_roll_rad);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.current_yaw_radps,
                                          &_ctx.data.current_pitch_radps,
                                          &_ctx.data.current_roll_radps);

    ins_drv_t::get_instance()->get_accel_b(&_ctx.data.current_x_accel,
                                           &_ctx.data.current_y_accel,
                                           &_ctx.data.current_z_accel);

    // 通常不用读取电机位置作为姿态反馈，改为使用 IMU 数据
    // // 读取 电机 数据作为反馈 (含 Offset)
    // _ctx.data.current_pitch_rad =
    //     _ctx.motor.pitch->get_current_position() - PITCH_OFFSET_RAD;
    // _ctx.data.current_pitch_radps = _ctx.motor.pitch->get_current_rotate();
    //
    // _ctx.data.current_yaw_rad =
    //     _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    // _ctx.data.current_yaw_radps = _ctx.motor.yaw->get_current_rotate();
}

void screw_gimbal_t::_gimbal_control(gimbal_context_t *ctx)
{
    // --- Pitch 串级控制 ---
    // 1. 位置环
    ctx->data.target_pitch_radps = ctx->pid.pitch_pos->calculate(
        ctx->data.target_pitch_rad, ctx->data.current_pitch_rad);

    // 2. 速度环
    ctx->data.out_pitch_torque = ctx->pid.pitch_spd->calculate(
        ctx->data.target_pitch_radps, ctx->data.current_pitch_radps);

    // --- Yaw 串级控制 ---
    // 1. 位置环
    ctx->data.target_yaw_radps = ctx->pid.yaw_pos->calculate(
        ctx->data.target_yaw_rad, ctx->data.current_yaw_rad);

    // 2. 速度环
    ctx->data.out_yaw_torque = ctx->pid.yaw_spd->calculate(
        ctx->data.target_yaw_radps, ctx->data.current_yaw_radps);

    // ctx->data.out_pitch_torque = 0.0f;
}

screw_gimbal_t::gimbal_context_t screw_gimbal_t::get_ctx() const
{
    return _ctx;
}

void screw_gimbal_t::_send_motor_command(gimbal_context_t *ctx)
{
    // ctx->motor.pitch->send_torque(ctx->data.out_pitch_torque);
    // ctx->motor.pitch->send_torque(ctx->data.out_gravity_torque +
    //                               ctx->data.out_pitch_torque);
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);

    ctx->motor.pitch->send_torque(0);
    // ctx->motor.yaw->send_torque(0);
}

void screw_gimbal_t::_communicate_chassis()
{
    std::array<uint8_t, 8> raw_data{};
    pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x102,
                                 raw_data);
    std::memcpy(&_ctx.data.current_chassis_pitch_rad,&raw_data,sizeof(float));
}

// =========================================================
// 状态机逻辑
// =========================================================

void screw_gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

} // namespace pyro