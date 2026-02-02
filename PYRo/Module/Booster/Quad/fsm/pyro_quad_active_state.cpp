#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::on_enter(owner *owner)
{
    change_state(&_homing_state);
}

void quad_booster_t::fsm_active_t::on_execute(owner *owner)
{

}

void quad_booster_t::fsm_active_t::on_exit(owner *owner)
{

}

} // namespace pyro