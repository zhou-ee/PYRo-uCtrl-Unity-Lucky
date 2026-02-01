#include "pyro_motor_base.h"

namespace pyro
{
motor_base_t::motor_base_t(can_hub_t::which_can which)
    : _which_can(which), _enable(false), _temperature(0), _current_position(0),
      _current_rotate(0), _current_torque(0)
{
    _can_drv = can_hub_t::get_instance()->hub_get_can_obj(which);
}

motor_base_t::~motor_base_t()
{
    // 基类析构函数实现（可为空）
}

int8_t motor_base_t::get_temperature(void)
{
    return _temperature;
}

float motor_base_t::get_current_position(void)
{
    return _current_position;
}

float motor_base_t::get_current_rotate(void)
{
    return _current_rotate;
}

float motor_base_t::get_current_torque(void)
{
    return _current_torque;
}

bool motor_base_t::is_enable(void)
{
    return _enable;
}

// Provide the virtual destructor definition so the vtable is emitted
motor_base_t::~motor_base_t()
{
    // base class cleanup (none currently)
}

} // namespace pyro
