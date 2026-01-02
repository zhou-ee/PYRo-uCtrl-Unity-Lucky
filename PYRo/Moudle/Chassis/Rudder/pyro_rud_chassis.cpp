// #include "pyro_rud_chassis.h"
// #include "pyro_dji_motor_drv.h"
// #include <cmath> // for abs
//
// namespace pyro
// {
//
// // =========================================================
// // 构造与析构
// // =========================================================
//
// rud_chassis_t::rud_chassis_t()
//     : chassis_base_t("rudder") // 调用基类构造
// {
//     // 【关键】实例化 FSM 对象，并赋值给基类的 _fsm 指针
//     // 注意：基类析构函数负责 delete 这个指针
//     _fsm = new rud_fsm_t();
// }
//
// rud_chassis_t::~rud_chassis_t()
// {
//     delete _kinematics;
//     for (int i = 0; i < 4; ++i)
//     {
//         delete _wheel_motor[i];
//         delete _rudder_motor[i];
//         delete _wheel_speed_pid[i];
//         delete _rudder_angle_pid[i];
//         delete _rudder_speed_pid[i];
//     }
//     delete _follow_angle_pid;
//     // _fsm 由 chassis_base_t::~chassis_base_t() 释放，此处无需 delete
// }
//
// // =========================================================
// // FSM 状态机逻辑实现
// // =========================================================
//
// // FSM 入口：决定初始状态
// void rud_chassis_t::rud_fsm_t::on_enter(Context *ctx)
// {
//     // 使用静态局部变量避免频繁 new/delete
//     static passive_state_t passive;
//     static active_state_t active;
//
//     // 默认进入被动状态，或者根据 ctx 内的标志位决定
//     change_state(&passive);
//
//     // 如果你想一启动就运行，也可以 change_state(&active);
//     // 或者可以通过检测遥控器开关来决定切换逻辑
//     // (这通常在 on_execute 中实现)
// }
//
// // --- Passive State (失能) ---
// void rud_chassis_t::passive_state_t::enter(Context *ctx)
// {
//     // 可以在这里重置 PID 积分项
// }
//
// void rud_chassis_t::passive_state_t::execute(Context *ctx)
// {
//     auto self = static_cast<rud_chassis_t*>(ctx);
//
//     // 即使在失能状态，也要更新反馈（为了显示UI或监控）
//     self->update_feedback();
//     self->stop_output(); // 发送零力矩
//
//     // 示例：状态切换逻辑
//     // if (self->_cmd.enable_flag) {
//     //     static active_state_t active;
//     //     request_switch(&active);
//     // }
// }
//
// void rud_chassis_t::passive_state_t::exit(Context *ctx) {}
//
// // --- Active State (使能) ---
// void rud_chassis_t::active_state_t::enter(Context *ctx) {}
//
// void rud_chassis_t::active_state_t::execute(Context *ctx)
// {
//     // 强转回派生类指针以访问私有成员和具体实现
//     auto self = static_cast<rud_chassis_t*>(ctx);
//
//     // 标准控制流
//     self->update_feedback();    // 1. 获取传感器数据
//     self->kinematics_solve();   // 2. 解算目标状态
//     self->chassis_control();    // 3. 计算 PID
//     self->power_control();      // 4. 功率限制
//     self->send_motor_command(); // 5. 发送 CAN
// }
//
// void rud_chassis_t::active_state_t::exit(Context *ctx)
// {
//     auto self = static_cast<rud_chassis_t*>(ctx);
//     self->stop_output(); // 退出运行状态时确保安全
// }
//
// // =========================================================
// // 业务逻辑实现 (Business Logic)
// // =========================================================
//
// void rud_chassis_t::init()
// {
//     _kinematics = new rudder_kin_t(0.35f, 0.35f);
//
//     // 电机初始化 (ID与CAN总线配置)
//     _wheel_motor[0]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1);
//     _wheel_motor[1]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can1);
//     _wheel_motor[2]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can1);
//     _wheel_motor[3]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can1);
//
//     _rudder_motor[0] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);
//     _rudder_motor[1] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can2);
//     _rudder_motor[2] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);
//     _rudder_motor[3] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can2);
//
//     // PID 初始化 (参数需根据实际调整)
//     for(int i=0; i<4; i++) {
//         _wheel_speed_pid[i]  = new pid_t(18.0f, 0.0f, 0.0f, 1.0f, 20.0f);
//         _rudder_angle_pid[i] = new pid_t(18.0f, 0.0f, 0.0f, 0.5f, 10.0f);
//         _rudder_speed_pid[i] = new pid_t(8.0f, 0.0f, 0.0f, 0.5f, 3.0f);
//     }
//     _follow_angle_pid = new pid_t(10.0f, 0.0f, 0.0f, 1.0f, 5.0f);
// }
//
// void rud_chassis_t::update_command()
// {
//     // 基类已经自动加锁并将 cmd 复制到 _cmd
//     // 这里可以进行死区处理或数据清洗
//     if (std::abs(_cmd.vx) < 0.01f) _cmd.vx = 0;
//     if (std::abs(_cmd.vy) < 0.01f) _cmd.vy = 0;
//     if (std::abs(_cmd.wz) < 0.01f) _cmd.wz = 0;
// }
//
// void rud_chassis_t::update_feedback()
// {
//     for (int i = 0; i < 4; ++i)
//     {
//         _wheel_motor[i]->update_feedback();
//         _rudder_motor[i]->update_feedback();
//
//         // 转换数据格式供 kinematics 使用
//         _current_states.modules[i].speed = _wheel_motor[i]->get_current_rotate() * dji_m3508_motor_drv_t::reciprocal_reduction_ratio;
//
//         _current_states.modules[i].angle = _rudder_motor[i]->get_current_position() - _rudder_offset[i];
//
//         _rudder_current_speed[i] = _rudder_motor[i]->get_current_rotate();
//     }
// }
//
// void rud_chassis_t::kinematics_solve()
// {
//     // 随动 PID 计算 (可选)
//     // if (_cmd.mode == FOLLOW && _cmd.yaw_err != 0) {
//     //     _cmd.wz = _follow_angle_pid->calculate(0, _cmd.yaw_err);
//     // }
//
//     // 使用基类的 _cmd 进行解算
//     _target_states = _kinematics->solve(_cmd.vx, _cmd.vy, _cmd.wz, _current_states);
// }
//
// void rud_chassis_t::chassis_control()
// {
//     for (int i = 0; i < 4; ++i)
//     {
//         // 轮子速度闭环
//         _wheel_output[i] = _wheel_speed_pid[i]->calculate(
//             _target_states.modules[i].speed, _current_states.modules[i].speed);
//
//         // 舵向角度串级闭环 (角度环 -> 速度环)
//         _rudder_target_speed[i] = _rudder_angle_pid[i]->calculate(
//             _target_states.modules[i].angle, _current_states.modules[i].angle);
//
//         _rudder_output[i] = _rudder_speed_pid[i]->calculate(
//             _rudder_target_speed[i], _rudder_current_speed[i]);
//     }
// }
//
// void rud_chassis_t::power_control()
// {
//     // 这里预留给功率控制逻辑
//     // 例如：如果总功率超限，按比例衰减 _wheel_output
// }
//
// void rud_chassis_t::send_motor_command()
// {
//     for (int i = 0; i < 4; ++i)
//     {
//         _wheel_motor[i]->send_torque(_wheel_output[i]);
//         _rudder_motor[i]->send_torque(_rudder_output[i]);
//     }
// }
//
// void rud_chassis_t::stop_output()
// {
//     for (int i = 0; i < 4; ++i)
//     {
//         _wheel_motor[i]->send_torque(0);
//         _rudder_motor[i]->send_torque(0);
//     }
// }
//
// } // namespace pyro