#include "pyro_dm_motor_drv.h"
#include "arm_math.h"

using namespace pyro;

dm_motor_drv_t* r_motor1;
dm_motor_drv_t* r_motor2;
dm_motor_drv_t* l_motor1;
dm_motor_drv_t* l_motor2;

extern "C" void pyro_chassis(void* argument)
{
    float r_theta1, r_theta2, l_theta1, l_theta2;
    float r_phi1, r_phi2, l_phi1, l_phi2;
    r_motor1 = new dm_motor_drv_t(0x01, 0x11, can_hub_t::can1);
    r_motor1->set_rotate_range(-54.0f, 54.0f);
    r_motor1->set_position_range(-12.5f, 12.5f);
    r_motor1->set_rotate_range(-45.0f, 45.0f);
    r_motor2 = new dm_motor_drv_t(0x02, 0x12, can_hub_t::can2);
    r_motor2->set_rotate_range(-54.0f, 54.0f);
    r_motor2->set_position_range(-12.5f, 12.5f);
    r_motor2->set_rotate_range(-45.0f, 45.0f);
    l_motor1 = new dm_motor_drv_t(0x03, 0x13, can_hub_t::can1);
    l_motor1->set_rotate_range(-54.0f, 54.0f);
    l_motor1->set_position_range(-12.5f, 12.5f);
    l_motor1->set_rotate_range(-45.0f, 45.0f);
    l_motor2 = new dm_motor_drv_t(0x04, 0x14, can_hub_t::can2);
    l_motor2->set_rotate_range(-54.0f, 54.0f);
    l_motor2->set_position_range(-12.5f, 12.5f);
    l_motor2->set_rotate_range(-45.0f, 45.0f);
    while(1)
    {
        r_motor1->enable();
        r_motor1->update_feedback();
        r_theta1 = -(r_motor1->get_current_position()) + 2.66f;
        r_motor2->enable();
        r_motor2->update_feedback();
        r_theta2 = -(r_motor2->get_current_position()) + 3.979092653f;

        arm_sin_f32(1);
        // r_phi1 = 2*atan((22506*arm_sin_f32(theta1(t)) - 22506*sin(theta2(t)) + (475976946 - 296955904*cos(theta1(t) - theta2(t)) - 179021042*cos(2*theta1(t) - 2*theta2(t)))^(1/2))/(22506*cos(theta1(t)) - 22506*cos(theta2(t)) + 18922*cos(theta1(t) - theta2(t)) - 18922))
        vTaskDelay(1);
    }
}