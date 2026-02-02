#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"


namespace pyro
{
void quad_booster_t::fsm_active_t::state_homing_t::enter(owner *owner)
{
    _homing_turnback_time = dwt_drv_t::get_current_ticks();
}

void quad_booster_t::fsm_active_t::state_homing_t::execute(owner *owner)
{


}

void quad_booster_t::fsm_active_t::state_homing_t::exit(owner *owner)
{

}
}