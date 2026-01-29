#include "pyro_hybrid_chassis.h"


namespace pyro
{

void hybrid_chassis_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.mecanum[0]->enable();
    owner->_ctx.motor.mecanum[1]->enable();
    owner->_ctx.motor.mecanum[2]->enable();
    owner->_ctx.motor.mecanum[3]->enable();
    owner->_ctx.motor.leg[0]->enable();
    owner->_ctx.motor.leg[1]->enable();
}

void hybrid_chassis_t::fsm_active_t::on_execute(owner *owner)
{
    if (hybrid_kin_t::drive_mode_t::CRUISING == owner->_ctx.cmd->drive_mode)
    {
        this->change_state(&_cruising_state);
    }
    else if (owner->_ctx.cmd->jump_mode == 1)
    {
        this->change_state(&_jumping_state);
    }
    else if (hybrid_kin_t::drive_mode_t::CLIMBING ==
             owner->_ctx.cmd->drive_mode)
    {
        this->change_state(&_climbing_state);
    }

    owner->_kinematics_solve();
}

void hybrid_chassis_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro