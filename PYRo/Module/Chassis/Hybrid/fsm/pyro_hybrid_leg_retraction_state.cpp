#include "pyro_hybrid_chassis.h"

namespace pyro
{
void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    enter(owner *owner)
{
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    execute(owner *owner)
{
    // for (int i = 0; i < 2; i++)
    // {
    //     owner->_ctx.data.target_leg_rad[i] = LEG_MIN_POS;
    // }
    //
    // owner->_leg_direct_control();
    // owner->_send_motor_command();
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    exit(owner *owner)
{
}


} // namespace pyro