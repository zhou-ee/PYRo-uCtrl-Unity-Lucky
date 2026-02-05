#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"


namespace pyro
{
void quad_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{
    owner->_ctx.data.target_trig_rad = owner->_ctx.data.current_trig_rad;
    owner->_ctx.data.target_trig_radps = 0;
}

void quad_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{

}

void quad_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{

}
}