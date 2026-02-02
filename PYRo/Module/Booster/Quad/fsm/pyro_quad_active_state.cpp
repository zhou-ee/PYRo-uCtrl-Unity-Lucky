#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::on_enter(owner *owner)
{
    change_state(&_homing_state);
}

void quad_booster_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->fric_on)
    {
        owner->_ctx.data.target_fric_mps[0] = owner->_ctx.cmd->fric2_mps;
        owner->_ctx.data.target_fric_mps[2] = -owner->_ctx.cmd->fric2_mps;


        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[3] = -owner->_ctx.cmd->fric1_mps;
        owner->_fric_control();
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[2] = 0.0f;
        owner->_ctx.data.target_fric_mps[3] = 0.0f;
        owner->_fric_control();
        for (int i = 0; i < 4; i++)
        {
            if (abs(owner->_ctx.data.current_fric_mps[i]) < 0.3f)
                owner->_ctx.data.out_fric_torque[i] = 0.0f;
        }
    }
    owner->_send_fric_command();
}

void quad_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro