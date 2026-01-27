#ifndef __PYRO_RUD_CHASSIS_H__
#define __PYRO_RUD_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_chassis_base.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"

namespace pyro
{

// 定义舵轮特有的命令结构（如果有额外参数）
struct cmd_rud_t final : cmd_base_t
{
    // 如果有舵轮特有的控制量（如锁舵模式标志位），可以在此添加
    cmd_rud_t() : cmd_base_t() {}
};

// 继承模板基类，传入具体的命令类型
class rud_chassis_t final : public chassis_base_t<cmd_rud_t>
{
  public:
    rud_chassis_t();
    ~rud_chassis_t() override;

  protected:
    // =========================================================
    // FSM 定义区域
    // =========================================================

    // 定义上下文类型别名，方便书写
    // 注意：这里是对象类型，不是指针！
    using Context = chassis_base_t<cmd_rud_t>;

    // 前向声明状态类


    // 1. 定义具体的 FSM
    // 继承时传入 Context (对象类型)，FSM 库内部会自动处理成 Context*
    class rud_fsm_t : public base_fsm_t<Context>
    {
        class active_state_t : public fsm_t<Context>
        {
        public:
            void on_enter(Context *owner) override;
            void on_execute(Context *owner) override;
            void on_exit(Context *owner) override;
        };
        class passive_state_t : public fsm_t<Context>
        {
        public:
            void on_enter(Context *ctx) override;
            void on_execute(Context *ctx) override;
            void on_exit(Context *ctx) override;
        };
    public:
        void on_enter(Context *ctx) override;
    };

    // 2. 定义 "主动控制" 状态 (Active)


    // 3. 定义 "被动/失能" 状态 (Passive)


    // =========================================================
    // 业务接口实现 (Override from chassis_base_t)
    // =========================================================
    void init() override;
    void update_command() override;   // 命令预处理
    void update_feedback() override;  // 反馈更新

    // 以下函数主要由 FSM 调用
    void kinematics_solve() override;   // 运动解算
    void chassis_control() override;    // 闭环控制
    void power_control() override;      // 功率限制
    void send_motor_command() override; // 发送指令

    // 辅助函数：全零输出
    void stop_output();

  private:
    // 资源句柄
    rudder_kin_t::rudder_states_t _target_states{};
    rudder_kin_t::rudder_states_t _current_states{};
    rudder_kin_t *_kinematics{};

    float _wheel_output[4]{};         // 轮毂电机力矩输出
    float _rudder_target_speed[4]{};  // 舵向速度目标
    float _rudder_current_speed[4]{}; // 舵向速度反馈
    float _rudder_output[4]{};        // 舵向电机力矩输出

    // 舵角零点偏移
    float _rudder_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    motor_base_t *_wheel_motor[4]{};  // FL, FR, BL, BR
    motor_base_t *_rudder_motor[4]{}; // Rudder Motors

    pid_t *_wheel_speed_pid[4]{};
    pid_t *_rudder_angle_pid[4]{};
    pid_t *_rudder_speed_pid[4]{};
    pid_t *_follow_angle_pid{};

    // 声明友元，允许状态机访问私有成员
    friend class active_state_t;
    friend class passive_state_t;
};

} // namespace pyro
#endif