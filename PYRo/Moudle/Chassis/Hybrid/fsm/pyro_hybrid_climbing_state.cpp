#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::state_climbing_t::enter(
    hybrid_chassis_t *owner)
{
    // owner->_ctx.motor.track[0]->enable();
    // owner->_ctx.motor.track[1]->enable();
}

void hybrid_chassis_t::fsm_active_t::state_climbing_t::execute(
    hybrid_chassis_t *owner)
{
    _chassis_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void hybrid_chassis_t::fsm_active_t::state_climbing_t::exit(
    hybrid_chassis_t *owner)
{
    // owner->_ctx.motor.track[0]->disable();
    // owner->_ctx.motor.track[1]->disable();
}

} // namespace pyro
