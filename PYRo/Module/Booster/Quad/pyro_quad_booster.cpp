#include "pyro_quad_booster.h"
#include <cmath> // For PI

namespace pyro
{

static float _r2rpm(const float r)
{
    return r * 9.5492966f;
}
static constexpr float PI_2 = 6.2831853f;

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
    _ctx = {};
}

void quad_booster_t::_init()
{
    // 1. 摩擦轮电机初始化 (CAN1 ID 1-4)
    _ctx.motor.fric_wheels[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    _ctx.motor.fric_wheels[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can1);
    _ctx.motor.fric_wheels[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can1);
    _ctx.motor.fric_wheels[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can1);


    // 摩擦轮速度 PID 初始化
    _ctx.pid.fric_pid[0] =
        new pid_t(10.0f, 0.1f, 0.0f, 1.0f, 20.0f, 100, 50, 4);
    _ctx.pid.fric_pid[1] =
        new pid_t(10.0f, 0.1f, 0.0f, 1.0f, 20.0f, 100, 50, 4);
    _ctx.pid.fric_pid[2] =
        new pid_t(10.0f, 0.1f, 0.0f, 1.0f, 20.0f, 100, 50, 4);
    _ctx.pid.fric_pid[3] =
        new pid_t(10.0f, 0.1f, 0.0f, 1.0f, 20.0f, 100, 50, 4);

    // 2. 拨弹电机初始化 (CAN1 ID 5)
    _ctx.motor.trigger_wheel =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_5, can_hub_t::can1);

    // 拨弹 PID 初始化
    _ctx.pid.trigger_pos_pid =
        new pid_t(12.0f, 0.0f, 0.0f, 0.0f, 10.0f, 200, 100, 4);
    _ctx.pid.trigger_spd_pid =
        new pid_t(10.0f, 0.0f, 0.0f, 5.0f, 20.0f, 100, 50, 4);
}

void quad_booster_t::_update_feedback()
{
    // 1. 摩擦轮反馈
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.cur_fric_rpm[i] =
            _r2rpm(_ctx.motor.fric_wheels[i]->get_current_rotate());
    }

    // 2. 拨弹反馈 (处理角度累加，变成连续角度)
    _ctx.motor.trigger_wheel->update_feedback();

    const float raw_angle =
        _ctx.motor.trigger_wheel->get_current_position(); // -PI ~ PI
    float delta = raw_angle - _ctx.data.last_trig_angle_raw;

    // 处理过零点 (-PI <-> PI)
    if (delta < -3.0f)
        delta += PI_2;
    else if (delta > 3.0f)
        delta -= PI_2;

    // 考虑减速比 (如19:1，M3508减速箱输出轴角度)
    // 注意：motor_drv 是否已经除过减速比？如果 get_current_position
    // 返回的是电机轴角度： delta *=
    // dji_m3508_motor_drv_t::reciprocal_reduction_ratio; 这里假设
    // get_current_position 已经是减速后的输出轴角度，或者我们在应用层乘比率
    // 假设需要乘比率：
    constexpr float ratio =
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio; // 1/19
    _ctx.data.cur_trig_angle_total += (delta * ratio);
    _ctx.data.last_trig_angle_raw = raw_angle;

    // 速度反馈
    _ctx.data.cur_trig_rpm =
        _r2rpm(_ctx.motor.trigger_wheel->get_current_rotate() * ratio);
    _ctx.data.cur_trig_torque = _ctx.motor.trigger_wheel->get_current_torque();
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_cmd[_read_index];

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void quad_booster_t::_run_control_loop()
{
    // 1. 摩擦轮控制 (单环)
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.tgt_fric_rpm[i], _ctx.data.cur_fric_rpm[i]);
    }

    // 2. 拨弹控制 (串级)
    // 外环：位置 -> 速度
    _ctx.data.trigger_spd_ref = _ctx.pid.trigger_pos_pid->calculate(
        _ctx.data.tgt_trig_angle, _ctx.data.cur_trig_angle_total);

    // 内环：速度 -> 电流(力矩)
    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.trigger_spd_ref, _ctx.data.cur_trig_rpm);
}

void quad_booster_t::_send_motor_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

} // namespace pyro