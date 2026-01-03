#include "pyro_hybrid_chassis.h"


namespace pyro
{

static float _mps_to_rpm(const float mps, const float radius)
{
    // v = w * r  -> w = v / r
    // RPM = w * 60 / 2pi
    if (radius < 1e-4f)
        return 0.0f;
    return (mps / radius) * 9.5492966f;
}

static float _radps_to_rpm(const float radps)
{
    // RPM = (w * 60) / (2 * pi)
    return radps * 9.5492966f;
}

// =========================================================
// 构造与初始化
// =========================================================

hybrid_chassis_t::hybrid_chassis_t()
    : chassis_base_t("hybrid", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data = {};
}

void hybrid_chassis_t::_init()
{
    _kinematics = new hybrid_kin_t(0.648f, 0.35f, 0.41f);

    _ctx.motor.mecanum[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can1); // FL
    _ctx.motor.mecanum[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can1); // FR
    _ctx.motor.mecanum[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BL
    _ctx.motor.mecanum[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // BR
    // _ctx.motor.track[0]   = new dm_motor_drv_t(0x31, 0x41, can_hub_t::can3);
    // _ctx.motor.track[1]   = new dm_motor_drv_t(0x32, 0x42, can_hub_t::can3);
    _ctx.motor.leg[0] = new dm_motor_drv_t(0x11, 0x21, can_hub_t::can3);
    _ctx.motor.leg[1] = new dm_motor_drv_t(0x12, 0x22, can_hub_t::can3);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[0])
    //     ->set_position_range(-PI, PI);
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[0])
    //     ->set_rotate_range(-50, 50);
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[0])
    //     ->set_torque_range(-10, 10);
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[1])
    //     ->set_position_range(-PI, PI);
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[1])
    //     ->set_rotate_range(-50, 50);
    // static_cast<dm_motor_drv_t *>(_ctx.motor.track[1])
    //     ->set_torque_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[0])->set_rotate_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[0])->set_torque_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[1])->set_rotate_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_ctx.motor.leg[1])->set_torque_range(-10, 10);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    _ctx.pid.mecanum_pid[0] =
        new pid_t(0.28f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _ctx.pid.mecanum_pid[1] =
        new pid_t(0.35f, 0.0007f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _ctx.pid.mecanum_pid[2] =
        new pid_t(0.37f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _ctx.pid.mecanum_pid[3] =
        new pid_t(0.36f, 0.0006f, 0.0001f, 1.0f, 20.0f, 20, 10, 4);

    // Track Speed Loop
    // for (auto &i : _ctx.pid.track_pid)
    // {
    //     i = new pid_t(0.02f, 0.0001f, 0.000001f, 0.5f, 10.0f, 30, 10, 4);
    // }

    // Leg Position Loop (Inner loop)
    // for (auto &i : _ctx.pid.leg_pos_pid)
    // {
    //     i = new pid_t(10.0f, 0.005f, 0.008f, 0.5f, 20.0f, 20, 10, 4);
    // }
    //
    // for (auto &i : _ctx.pid.leg_spd_pid)
    // {
    //     i = new pid_t(10.0f, 0.005f, 0.008f, 0.5f, 7.0f, 20, 10, 4);
    // }
    for (auto &i : _ctx.pid.leg_pos_pid)
    {
        i = new pid_t(15.0f, 0.005f, 0.008f, 0.5f, 20.0f, 20, 10, 4);
    }

    for (auto &i : _ctx.pid.leg_spd_pid)
    {
        i = new pid_t(1.2f, 0.005f, 0.008f, 0.5f, 7.0f, 20, 10, 4);
    }



    _ctx.pid.balance_pid =
        new pid_t(3.14f, 0.5f, 0.1f, 0.0f,
                  0.05f); // Output is radians (position offset)

    _ctx.hardware.power_meter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.power.data           = new powermeter_data();
}

void hybrid_chassis_t::_update_feedback()
{
    _ctx.motor.mecanum[0]->update_feedback();
    _ctx.motor.mecanum[1]->update_feedback();
    _ctx.motor.mecanum[2]->update_feedback();
    _ctx.motor.mecanum[3]->update_feedback();
    _ctx.motor.leg[0]->update_feedback();
    _ctx.motor.leg[1]->update_feedback();
    // _ctx.motor.track[0]->update_feedback();
    // _ctx.motor.track[1]->update_feedback();

    _ctx.data.current_wheel_rpm[0] =
        _radps_to_rpm(_ctx.motor.mecanum[0]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_wheel_rpm[1] =
        _radps_to_rpm(_ctx.motor.mecanum[1]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_wheel_rpm[2] =
        _radps_to_rpm(_ctx.motor.mecanum[2]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _ctx.data.current_wheel_rpm[3] =
        _radps_to_rpm(_ctx.motor.mecanum[3]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    // 2. 两条履带的 RPM、腿角度、腿角速度
    // _ctx.data.current_track_rpm[0] =
    //     _radps_to_rpm(_ctx.motor.track[0]->get_current_rotate());
    // _ctx.data.current_track_rpm[1] =
    //     _radps_to_rpm(_ctx.motor.track[1]->get_current_rotate());

    _ctx.data.current_leg_rad[0]   = _ctx.motor.leg[0]->get_current_position();
    _ctx.data.current_leg_radps[0] = _ctx.motor.leg[0]->get_current_rotate();
    _ctx.data.current_leg_rad[1]   = _ctx.motor.leg[1]->get_current_position();
    _ctx.data.current_leg_radps[1] = _ctx.motor.leg[1]->get_current_rotate();

    debug_data.debug_leg_torque[0] = _ctx.motor.leg[0]->get_current_torque();
    debug_data.debug_leg_torque[1] = _ctx.motor.leg[1]->get_current_torque();
}

void hybrid_chassis_t::_kinematics_solve()
{
    static hybrid_kin_t::hybrid_speeds_t solved_speeds_mps{};

    solved_speeds_mps = _kinematics->solve(_ctx.cmd->vx, _ctx.cmd->vy,
                                           _ctx.cmd->wz, _ctx.cmd->drive_mode);
    _ctx.data.target_wheel_rpm[0] =
        _mps_to_rpm(solved_speeds_mps.mec_fl, MEC_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -_mps_to_rpm(solved_speeds_mps.mec_fr, MEC_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        _mps_to_rpm(solved_speeds_mps.mec_bl, MEC_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -_mps_to_rpm(solved_speeds_mps.mec_br, MEC_RADIUS);

    // _ctx.data.target_track_rpm[0] =
    //     _mps_to_rpm(solved_speeds_mps.track_l, TRACK_RADIUS);
    // _ctx.data.target_track_rpm[1] =
    //     -_mps_to_rpm(solved_speeds_mps.track_r, TRACK_RADIUS);

    _ctx.data.target_leg_rad[0] += _ctx.cmd->wy;
    _ctx.data.target_leg_rad[1] -= _ctx.cmd->wy;
    // if (_ctx.data.target_leg_rad[0] > LEG_EXTEND_POS)
    // {
    //     _ctx.data.target_leg_rad[0] = LEG_EXTEND_POS;
    // }
    // else if (_ctx.data.target_leg_rad[0] < LEG_RETRACT_POS)
    // {
    //     _ctx.data.target_leg_rad[0] = LEG_RETRACT_POS;
    // }
    // if (_ctx.data.target_leg_rad[1] < -LEG_EXTEND_POS)
    // {
    //     _ctx.data.target_leg_rad[1] = -LEG_EXTEND_POS;
    // }
    // else if (_ctx.data.target_leg_rad[1] > -LEG_RETRACT_POS)
    // {
    //     _ctx.data.target_leg_rad[1] = -LEG_RETRACT_POS;
    // }
}

void hybrid_chassis_t::_chassis_control(hybrid_context_t *ctx)
{
    // 1. Mecanum Wheels PID (Speed Loop)
    for (int i = 0; i < 4; i++)
    {
        ctx->data.out_mecanum_torque[i] = ctx->pid.mecanum_pid[i]->calculate(
            ctx->data.target_wheel_rpm[i], ctx->data.current_wheel_rpm[i]);
    }

    // 2. Track PID (Speed Loop)
    // for (int i = 0; i < 2; i++)
    // {
    //     ctx->data.out_track_torque[i] = ctx->pid.track_pid[i]->calculate(
    //         ctx->data.target_track_rpm[i], ctx->data.current_track_rpm[i]);
    // }

    static float leg_pos_target_radps[2] = {0.0f, 0.0f};
    for (int i = 0; i < 2; i++)
    {
        leg_pos_target_radps[i] = ctx->pid.leg_pos_pid[i]->calculate(
            ctx->data.target_leg_rad[i], ctx->data.current_leg_rad[i]);
        ctx->data.out_leg_torque[i] = ctx->pid.leg_spd_pid[i]->calculate(
            leg_pos_target_radps[i], ctx->data.current_leg_radps[i]);
    }
}

void hybrid_chassis_t::_send_motor_command(hybrid_context_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        ctx->motor.mecanum[i]->send_torque(ctx->data.out_mecanum_torque[i]);
    }

    // for (int i = 0; i < 2; i++)
    // {
    //     ctx->motor.track[i]->send_torque(ctx->data.out_track_torque[i]);
    // }

    for (int i = 0; i < 2; i++)
    {
        ctx->motor.leg[i]->send_torque(ctx->data.out_leg_torque[i]);
    }
    // for (int i = 0; i < 4; i++)
    // {
    //     ctx->motor.mecanum[i]->send_torque(0);
    // }
    //
    // for (int i = 0; i < 2; i++)
    // {
    //     ctx->motor.track[i]->send_torque(0);
    // }
    //
    // for (int i = 0; i < 2; i++)
    // {
    //     ctx->motor.leg[i]->send_torque(0);
    // }
}

// =========================================================
// 核心运行时
// =========================================================
void hybrid_chassis_t::_fsm_execute()
{
    // 1. 数据刷新
    _ctx.cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::ZERO_FORCE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    // 2. 反馈更新
    // _pure_update_feedback(_ctx.motor, _ctx.data);

    // 3. 状态机运行
    // 父级 execute 内部会自动调用 fetch_request 处理子状态的切换请求
    _main_fsm.execute(this);

    // 4. 硬件输出
    // _pure_hw_write(_ctx.motor, _ctx.data);
}


} // namespace pyro