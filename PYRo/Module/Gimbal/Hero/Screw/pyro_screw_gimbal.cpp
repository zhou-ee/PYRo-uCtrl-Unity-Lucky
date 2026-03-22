#include "pyro_screw_gimbal.h"
#include "pyro_ins.h"
#include "pyro_com_canrx.h"
#include "pyro_algo_common.h"
#include "screw_config.h"
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
    _ctx.pid   = _module_deps.pid_deps;
    pyro::can_rx_drv_t::subscribe(can_hub_t::which_can::can1, 0x102);
    pyro::can_rx_drv_t::subscribe(can_hub_t::which_can::can1, 0x103);
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

    float current_angle =
    _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    current_angle = loop_fp32_constrain(current_angle, -PI, PI);
    _ctx.data.relative_yaw_rad =
        loop_fp32_constrain(current_angle, -PI, PI);


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
    ins_drv_t::get_instance()->get_quaternion(
        &_ctx.data.gimbal_q[0], &_ctx.data.gimbal_q[1], &_ctx.data.gimbal_q[2],
        &_ctx.data.gimbal_q[3]);

    _communicate_chassis();
    _calculate_relative_angles();

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
        ctx->data.target_pitch_radps, ctx->data.current_pitch_rad);

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
    ctx->motor.pitch->send_torque(0);
    // ctx->motor.pitch->send_torque(ctx->data.out_gravity_torque +
    //                               ctx->data.out_pitch_torque);
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);

    // ctx->motor.pitch->send_torque(0);
    // ctx->motor.yaw->send_torque(0);
}

void screw_gimbal_t::_communicate_chassis()
{
    std::array<uint8_t, 8> raw_data1{};
    pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x102,
                                 raw_data1);
    std::memcpy(&_ctx.data.current_chassis_pitch_rad, &raw_data1,
                sizeof(float));
    std::array<uint8_t, 8> raw_data2{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x103,
                                     raw_data2))
    {
        // 强制转换为 int16_t 指针，方便按索引访问
        auto *src              = reinterpret_cast<int16_t *>(raw_data2.data());
        // 逐个还原并放回 float 数组
        _ctx.data.chassis_q[0] = static_cast<float>(src[0]) / 32767.0f; // q0
        _ctx.data.chassis_q[1] = static_cast<float>(src[1]) / 32767.0f; // q1
        _ctx.data.chassis_q[2] = static_cast<float>(src[2]) / 32767.0f; // q2
        _ctx.data.chassis_q[3] = static_cast<float>(src[3]) / 32767.0f; // q3
    }
}

void screw_gimbal_t::_calculate_relative_angles()
{
    // 1. 提取原始四元数
    const float cw = _ctx.data.chassis_q[0], cx = _ctx.data.chassis_q[1],
                cy = _ctx.data.chassis_q[2], cz = _ctx.data.chassis_q[3];

    const float gw = _ctx.data.gimbal_q[0], gx = _ctx.data.gimbal_q[1],
                gy = _ctx.data.gimbal_q[2], gz = _ctx.data.gimbal_q[3];

    // ====================================================================
    // 步骤 A：Yaw 轴坐标系动态对齐 (消除开机错位与陀螺仪零漂)
    // ====================================================================

    // 从四元数中反解出两个 IMU 认为的世界 Yaw 角
    float chassis_yaw_imu = std::atan2(2.0f * (cw * cz + cx * cy), 1.0f - 2.0f * (cy * cy + cz * cz));
    float gimbal_yaw_imu  = std::atan2(2.0f * (gw * gz + gx * gy), 1.0f - 2.0f * (gy * gy + gz * gz));

    // 计算当前的坐标系漂移误差: Error = 云台测算Yaw - 底盘测算Yaw - 真实机械Yaw
    float raw_yaw_error = gimbal_yaw_imu - chassis_yaw_imu - _ctx.data.relative_yaw_rad;

    // 限制误差在 [-PI, PI] 范围内，防止角度突变 (使用你代码中已有的工具函数)
    raw_yaw_error = pyro::loop_fp32_constrain(raw_yaw_error, -PI, PI);

    // 低通滤波，让坐标系缓慢平滑地对齐，避免高频噪声引起云台抖动
    static float filtered_yaw_error = 0.0f;
    filtered_yaw_error = filtered_yaw_error * 0.99f + raw_yaw_error * 0.01f;

    // 构造只绕 Z 轴旋转的补偿四元数 (对齐矩阵)
    float half_err = filtered_yaw_error * 0.5f;
    float comp_w = std::cos(half_err);
    float comp_z = std::sin(half_err);

    // 将补偿四元数乘到底盘四元数上： Q_aligned = Q_comp * Q_chassis
    // 这一步相当于在空间中把底盘的坐标系强行“拧”过来，与云台坐标系对齐
    float aw = comp_w * cw - comp_z * cz;
    float ax = comp_w * cx - comp_z * cy;
    float ay = comp_w * cy + comp_z * cx;
    float az = comp_w * cz + comp_z * cw;

    // ====================================================================
    // 步骤 B：使用对齐后的底盘四元数 (aw, ax, ay, az) 提取相对 Pitch
    // ====================================================================

    // 计算对齐后底盘旋转矩阵的转置 (RcT) 的第三行元素
    float RcT_row2_0 = 2.0f * (ax * az + aw * ay);
    float RcT_row2_1 = 2.0f * (ay * az - aw * ax);
    float RcT_row2_2 = 1.0f - 2.0f * (ax * ax + ay * ay);

    // 云台旋转矩阵 (Rg) 的相关列元素保持不变 (使用原始 gw, gx, gy, gz)
    float Rg_col0_0 = 1.0f - 2.0f * (gy * gy + gz * gz);
    float Rg_col0_1 = 2.0f * (gx * gy + gw * gz);
    float Rg_col0_2 = 2.0f * (gx * gz - gw * gy);

    float Rg_col2_0 = 2.0f * (gx * gz + gw * gy);
    float Rg_col2_1 = 2.0f * (gy * gz - gw * gx);
    float Rg_col2_2 = 1.0f - 2.0f * (gx * gx + gy * gy);

    // 矩阵相乘提取相对 Pitch 关键元素
    float r31 = RcT_row2_0 * Rg_col0_0 + RcT_row2_1 * Rg_col0_1 + RcT_row2_2 * Rg_col0_2;
    float r33 = RcT_row2_0 * Rg_col2_0 + RcT_row2_1 * Rg_col2_1 + RcT_row2_2 * Rg_col2_2;

    // 输出极其纯净的相对 Pitch
    _ctx.data.relative_pitch_rad = std::atan2(-r31, r33);
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