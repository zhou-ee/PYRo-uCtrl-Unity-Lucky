#include "pyro_hybrid_chassis.h"

namespace pyro
{
static uint8_t jump_phase = 0;
void hybrid_chassis_t::fsm_active_t::state_jumping_t::enter(
    hybrid_chassis_t *owner)
{
    jump_phase = 1;
}

void hybrid_chassis_t::fsm_active_t::state_jumping_t::execute(
    hybrid_chassis_t *owner)
{

    _chassis_control(&owner->_ctx);

    const float jump_spd_limit = 12.0f;

    if (jump_phase == 1)
    {
        if (owner->_ctx.data.current_leg_radps[0] < jump_spd_limit &&
            owner->_ctx.data.current_leg_rad[0] < (LEG_EXTEND_POS - 0.1f))
        {
            owner->_ctx.data.out_leg_torque[0] = 8.0f;
        }
        else
        {
            jump_phase = 2;
        }

        if (owner->_ctx.data.current_leg_radps[1] > -jump_spd_limit &&
            owner->_ctx.data.current_leg_rad[1] > (-LEG_EXTEND_POS + 0.1f))
        {
            owner->_ctx.data.out_leg_torque[1] = -8.0f;
        }
        else
        {
            jump_phase = 2;
        }
    }
    if (jump_phase == 2)
    {
        owner->_ctx.data.target_leg_rad[0] = LEG_RETRACT_POS;
        owner->_ctx.data.target_leg_rad[1] = -LEG_RETRACT_POS;
    }

    _send_motor_command(&owner->_ctx);
}

void hybrid_chassis_t::fsm_active_t::state_jumping_t::exit(
    hybrid_chassis_t *owner)
{
    jump_phase = 0;
}


} // namespace pyro