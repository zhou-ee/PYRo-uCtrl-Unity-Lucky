#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_enter(owner *owner)
{
    // 进入爬坡模式，重新初始化所有驱动机构的 PID
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_execute(owner *owner)
{
    // 1. VMC 控制逻辑
    owner->_leg_vmc();

    // 1. 麦轮速度环控制 (提供前轮牵引力)
    owner->_mecanum_control();

    // 2. 履带速度环控制 (履带正式介入，提供主要越障/爬坡推进力)
    owner->_track_control();


    // 2. 统一发送所有电机指令
    owner->_send_motor_command();

    // if (owner->_ctx.cmd->leg_retract)
    // {
    //     change_state(&leg_retraction_state);
    // }
    // else
    // {
    //     change_state(&track_climbing_state);
    // }
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_exit(owner *owner)
{
    // 退出爬坡模式时的清理工作（当前可留空）
}

} // namespace pyro