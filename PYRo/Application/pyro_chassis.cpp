#include "pyro_dm_motor_drv.h"
#include "pyro_common.h"
#include "arm_math.h"
#include "pyro_rc_hub.h"

using namespace pyro;

dm_motor_drv_t* r_motor1;
dm_motor_drv_t* r_motor2;
dm_motor_drv_t* l_motor1;
dm_motor_drv_t* l_motor2;
float r_theta1, r_theta2, l_theta1, l_theta2;
float r_phi1, r_phi2, l_phi1, l_phi2;
float r_alpha, l_alpha;
float r_l, l_l;
float r_T[2],r_F[2],l_T[2],l_F[2],T_val[4];
arm_matrix_instance_f32 T;
rc_drv_t *dr16_drv;



void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T);


extern "C" void pyro_chassis(void* argument)
{
    dr16_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);

    arm_mat_init_f32(&T, 2, 2, T_val);
    
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

        float temp1, temp2;
        // update the angle of motor
        r_motor1->enable();
        r_motor1->update_feedback();
        r_theta1 = -(r_motor1->get_current_position()) + 2.66f;
        r_motor2->enable();
        r_motor2->update_feedback();
        r_theta2 = -(r_motor2->get_current_position()) + 3.979092653f;

        // caculate other angle
        arm_sqrt_f32(  (475976946.0f - 296955904*arm_cos_f32(r_theta1 - r_theta2) - 179021042.0f*arm_cos_f32(2*r_theta1 - 2*r_theta2)), &temp1);
        arm_atan2_f32((22506*arm_sin_f32(r_theta1) - 22506*arm_sin_f32(r_theta2)+temp1) /(22506*arm_cos_f32(r_theta1) - 22506*arm_cos_f32(r_theta2) + 18922*arm_cos_f32(r_theta1 - r_theta2) - 18922),1.0f,&temp2);
        r_phi1 = (2 * temp2);
        arm_atan2_f32((22506*arm_sin_f32(r_theta1) - 22506*arm_sin_f32(r_theta2)+temp1) /(22506*arm_cos_f32(r_theta1) - 22506*arm_cos_f32(r_theta2) - 18922*arm_cos_f32(r_theta1 - r_theta2) + 18922),1.0f,&temp2);
        r_phi2 = (2 * temp2);  
        arm_atan2_f32((236976927.0f*arm_sin_f32(r_phi1))/946100000.0f + (21059*arm_sin_f32(r_theta1))/100000, (236976927.0f*arm_cos_f32(r_phi1))/946100000.0f + (21059*arm_cos_f32(r_theta1))/100000, &r_alpha);
        
        // caculate the position in polar coordinates
        temp1 = (236976927.0f*arm_cos_f32(r_phi1))/946100000.0f + (21059*arm_cos_f32(r_theta1))/100000;
        temp1 = temp1 * temp1;
        temp2 = (236976927.0f*arm_sin_f32(r_phi1))/946100000.0f + (21059*arm_sin_f32(r_theta1))/100000;
        temp2 = temp2 * temp2;
        arm_sqrt_f32(temp1 + temp2, &r_l);

        // VMC transform
        r_F[0] = 1.0f;
        r_F[1] = 1.0f;
        update_transform_matrix(r_phi1, r_phi2, 
                              r_theta1, r_theta2,
                               r_alpha, r_l, T.pData);
        arm_mat_vec_mult_f32(&T, r_F, r_T);

        vTaskDelay(1);

    }
}

void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T)
{
    T[0] = (21059*arm_cos_f32(phi2)*arm_sin_f32(alpha)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_cos_f32(alpha)*arm_sin_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2));
    T[1] = -((21059*arm_cos_f32(alpha)*arm_cos_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_sin_f32(alpha)*arm_sin_f32(phi2)*arm_sin_f32(phi1 - theta1))/(100000*arm_sin_f32(phi1 - phi2)))/l;
    T[2] = (21059*arm_cos_f32(alpha)*arm_sin_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_cos_f32(phi1)*arm_sin_f32(alpha)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2));
    T[3] = ((21059*arm_cos_f32(alpha)*arm_cos_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)) - (21059*arm_sin_f32(alpha)*arm_sin_f32(phi1)*arm_sin_f32(phi2 - theta2))/(100000*arm_sin_f32(phi1 - phi2)))/l;
}

void update_rc(void)
{
    read_scope_lock rc_read_lock(dr16_drv->get_lock());
    auto rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(
                                                            dr16_drv->read());
}