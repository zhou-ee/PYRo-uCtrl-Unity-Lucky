#include "pyro_mec_chassis.h"

namespace pyro
{
void mec_chassis_t::state_passive_t::enter(owner *owner)
{
    // 清零目标
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0;
        owner->_ctx.data.out_wheel_torque[i] = 0;
    }
}

void mec_chassis_t::state_passive_t::execute(owner *owner)
{
    // 持续输出 0 力矩
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0;
    }
    owner->_send_motor_command(&owner->_ctx);
}

void mec_chassis_t::state_passive_t::exit(owner *owner)
{
    // 复位 PID 积分项等，防止突变
    for (auto *pid : owner->_ctx.pid.wheel_pid)
    {
        pid->clear();
    }
}
} // namespace pyro