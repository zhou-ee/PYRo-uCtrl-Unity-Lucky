#include "pyro_direct_gimbal.h"

namespace pyro
{
// =========================================================
// 状态实现 - Active (控制模式)
// =========================================================

void direct_gimbal_t::state_active_t::enter(owner *owner)
{
}

void direct_gimbal_t::state_active_t::execute(owner *owner)
{
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
    owner->_ctx.data.target_yaw_rad   += owner->_ctx.cmd->yaw_delta_angle;

    _gimbal_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void direct_gimbal_t::state_active_t::exit(owner *owner)
{
}
} // namespace pyro