#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::state_passive_t::enter(owner *owner)
{
    for (int i = 0; i < 4; i++) 
    {
        owner->_ctx.pid.fric_pid[i]->clear();
    }

    owner->_ctx.pid.trigger_pos_pid->clear();
    owner->_ctx.pid.trigger_spd_pid->clear();
}

void quad_booster_t::state_passive_t::execute(owner *owner)
{
    // 持续输出0
    for (int i = 0; i < 4; i++) owner->_ctx.data.out_fric_torque[i] = 0;
    owner->_ctx.data.out_trig_torque = 0;

    owner->_send_motor_command();
}

void quad_booster_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro