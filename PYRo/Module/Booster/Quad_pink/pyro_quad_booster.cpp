#include "pyro_quad_booster.h"
#include "pyro_algo_common.h"
#include "pyro_com_canrx.h"

#include <cmath>

namespace pyro
{

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
    _ctx = {};
}

status_t quad_booster_t::_init()
{
    // 1. 摩擦轮电机初始化
    _ctx.motor.fric_wheels[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);
    _ctx.motor.fric_wheels[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can2);
    _ctx.motor.fric_wheels[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);
    _ctx.motor.fric_wheels[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can2);

    // 摩擦轮 PID
    _ctx.pid.fric_pid[0] = new pid_t(6.40f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[1] =
        new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[2] =
        new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[3] = new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);

    // 2. 拨弹电机初始化
    _ctx.motor.trigger_wheel =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_5, can_hub_t::can3);

    // 拨弹 PID
    _ctx.pid.trigger_pos_pid =
        new pid_t(10.2f, 0.03f, 0.005f, 1.0f, 15.0f, 200, 100, 4);
    _ctx.pid.trigger_spd_pid =
        new pid_t(3.6f, 0.02f, 0.005f, 2.0f, 20.0f, 200, 100, 4);
    // 14
    //  重置数据
    _ctx.data.last_rotor_rad = 0.0f; // 默认从0开始比较
    _ctx.data.total_trig_rad = 0.0f;

    // 3. 弹速控制初始化
    can_rx_drv_t::subscribe(can_hub_t::can3, 0x102);
    _ctx.pid.ball_speed_pid =
        new pid_t(0.25f, 0.0f, 0.005f, 0.0f, 2.0f);

    return PYRO_OK;
}

float quad_booster_t::_normalize_angle(float angle)
{
    // 归一化到 [-PI, PI]
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

void quad_booster_t::_update_feedback()
{
    // 1. 摩擦轮反馈
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_mps[i] =
            _ctx.motor.fric_wheels[i]->get_current_rotate() * FRIC1_RADIUS;
    }

    // 2. 拨弹反馈
    _ctx.motor.trigger_wheel->update_feedback();

    constexpr float reciprocal_ratio =
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio;

    // --- A. 速度反馈 ---
    _ctx.data.current_trig_radps =
        _ctx.motor.trigger_wheel->get_current_rotate() * reciprocal_ratio;

    // --- B. 扭矩反馈 ---
    _ctx.data.current_trig_torque =
        _ctx.motor.trigger_wheel->get_current_torque();

    // --- C. 位置反馈 (移除首帧检查) ---
    // 获取当前转子角度 (范围 -PI 到 PI)
    const float now_rotor_rad =
        _ctx.motor.trigger_wheel->get_current_position();

    // 计算转子角度差值 (当前 - 上一次)
    float delta_rotor = now_rotor_rad - _ctx.data.last_rotor_rad;

    // 处理过零点问题 (最短路径原则)
    if (delta_rotor > PI)
    {
        delta_rotor -= 2.0f * PI;
    }
    else if (delta_rotor < -PI)
    {
        delta_rotor += 2.0f * PI;
    }

    // 累积到输出轴总角度
    _ctx.data.total_trig_rad += delta_rotor * reciprocal_ratio;

    // 更新上一帧数据
    _ctx.data.last_rotor_rad   = now_rotor_rad;

    // --- D. 归一化输出角度 (-PI ~ PI) ---
    _ctx.data.current_trig_rad = _normalize_angle(_ctx.data.total_trig_rad);
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void quad_booster_t::_speed_contorl()
{
    std::array<uint8_t, 8> raw_data{};

    // 仅在成功接收到新弹速的这一帧，才进行闭环计算
    if (can_rx_drv_t::get_data(pyro::can_hub_t::can3, 0x102, raw_data))
    {
        // 1. 更新弹速历史数据
        _ctx.shoot_data.ball_speed[2] = _ctx.shoot_data.ball_speed[1];
        _ctx.shoot_data.ball_speed[1] = _ctx.shoot_data.ball_speed[0];
        _ctx.shoot_data.ball_speed[0] =
            *reinterpret_cast<float *>(raw_data.data());

        for (int i = 0; i < 3; i++)
        {
            if (_ctx.shoot_data.ball_speed[i] == 0.0f)
            {
                _ctx.shoot_data.ball_speed[i] = _ctx.cmd->target_speed;
            }
        }

        // 2. 确保目标弹速有效，避免启动时出现误动作
        if (_ctx.cmd->target_speed > 7.5f)
        {
            // --- A. 定义近期弹速的权重 ---
            // 越新的弹速参考价值越大
            constexpr float w0 = 0.72f; // 最新一发
            constexpr float w1 = 0.21f; // 上一发
            constexpr float w2 = 0.07f; // 上上发

            // --- B. 计算带符号的均方误差 ---
            float e0 = _ctx.shoot_data.ball_speed[0] - _ctx.cmd->target_speed;
            float e1 = _ctx.shoot_data.ball_speed[1] - _ctx.cmd->target_speed;
            float e2 = _ctx.shoot_data.ball_speed[2] - _ctx.cmd->target_speed;

            // 采用 e * |e| 保留误差方向 (加速或减速)
            float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                        (w1 * e1 * std::abs(e1)) +
                                        (w2 * e2 * std::abs(e2));

            // --- C. PID 计算速度增量 ---
            // 由于 signed_weighted_mse 本身已经是误差值，直接将其作为
            // target，current 设为 0
            float speed_increment =
                _ctx.pid.ball_speed_pid->calculate(0.0f, signed_weighted_mse);

            // --- D. 累加到 fric1 的基础转速上 ---
            _ctx.shoot_data.fric1_mps += speed_increment;

            // --- E. 安全限幅 (非常重要) ---
            // 避免闭环异常导致单侧摩擦轮转速过高或过低，导致卡弹或弹道严重偏斜
            // 这里的限幅值请根据你实际的摩擦轮物理极限进行调整
            constexpr float MAX_FRIC1_MPS = 16.0f;
            constexpr float MIN_FRIC1_MPS = 9.0f;

            if (_ctx.shoot_data.fric1_mps > MAX_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MAX_FRIC1_MPS;
            }
            else if (_ctx.shoot_data.fric1_mps < MIN_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MIN_FRIC1_MPS;
            }
        }
    }
}

void quad_booster_t::_fric_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void quad_booster_t::_trigger_position_control()
{
    const float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;
    // 处理过零点问题，选择最短路径
    if (error > PI)
    {
        _ctx.data.target_trig_rad -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        _ctx.data.target_trig_rad += 2.0f * PI;
    }

    // 拨弹 PID 计算
    // 使用归一化后的 -PI~PI 角度进行控制
    _ctx.data.target_trig_radps = _ctx.pid.trigger_pos_pid->calculate(
        _ctx.data.target_trig_rad, _ctx.data.current_trig_rad);

    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void quad_booster_t::_trigger_speed_control()
{
    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void quad_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
}

void quad_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

} // namespace pyro