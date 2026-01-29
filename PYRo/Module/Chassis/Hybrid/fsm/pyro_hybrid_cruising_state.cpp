#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::state_cruising_t::enter(
    hybrid_chassis_t *owner)
{
    float current_leg_avg_rad = (owner->_ctx.data.current_leg_rad[0] -
                                 owner->_ctx.data.current_leg_rad[1]) /
                                2.0f;
    // if (current_leg_avg_rad < LEG_RETRACT_POS)
    // {
    //     current_leg_avg_rad = LEG_RETRACT_POS;
    // }

    owner->_ctx.data.target_leg_rad[0] = current_leg_avg_rad;
    owner->_ctx.data.target_leg_rad[1] = -current_leg_avg_rad;
}

void hybrid_chassis_t::fsm_active_t::state_cruising_t::execute(
    hybrid_chassis_t *owner)
{
    // 3. 运行闭环控制
    _chassis_control(&owner->_ctx);

    // 4. 输出到硬件
    _send_motor_command(&owner->_ctx);
}

void hybrid_chassis_t::fsm_active_t::state_cruising_t::exit(
    hybrid_chassis_t *owner)
{
    // 退出巡航模式
}

} // namespace pyro
