#include "pyro_mec_chassis.h"

namespace pyro
{
// --- Active State ---
void mec_chassis_t::state_active_t::enter(owner *owner)
{
    // 无特殊操作
}

void mec_chassis_t::state_active_t::execute(owner *owner)
{
    // 1. 运动学解算 (Input: cmd -> Output: target_rpm)
    owner->_kinematics_solve();

    // 2. 闭环控制 (Input: target_rpm, current_rpm -> Output: torque)
    owner->_chassis_control(&owner->_ctx);

    // 3. 发送波形
    owner->_send_motor_command(&owner->_ctx);
}

void mec_chassis_t::state_active_t::exit(owner *owner)
{
    // 无特殊操作
}
} // namespace pyro