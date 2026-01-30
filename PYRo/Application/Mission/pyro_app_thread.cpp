#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_direct_gimbal.h"

pyro::mec_chassis_t *mec_chassis_ptr             = nullptr;
pyro::mec_cmd_t *mec_cmd_ptr                     = nullptr;
pyro::direct_gimbal_t *direct_gimbal_ptr         = nullptr;
pyro::direct_gimbal_cmd_t *direct_gimbal_cmd_ptr = nullptr;
pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

extern "C"
{
    void chassis_rc2cmd(void const *rc_ctrl)
    {
        pyro::scoped_mutex_t lock(mec_chassis_ptr->get_mutex());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        // 1. 检查右侧开关状态，如果是 DOWN，则进入 ZERO_FORCE 模式
        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
        {
            mec_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ZERO_FORCE;
            mec_cmd_ptr->vx   = 0;
            mec_cmd_ptr->vy   = 0;
            mec_cmd_ptr->wz   = 0;
            return;
        }
        mec_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        // 3. 映射摇杆数据
        // 注意：根据旧代码，vx = -Right_Y, vy = Right_X, wz = -Left_X, wy =
        // Left_Y * 0.002 新接口中，通道名称为 ch_rx, ch_ry, ch_lx, ch_ly
        mec_cmd_ptr->vx   = -p_ctrl->rc.ch_ry;
        mec_cmd_ptr->vy   = p_ctrl->rc.ch_rx;
    }

    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        pyro::scoped_mutex_t lock(direct_gimbal_ptr->get_mutex());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        // 1. 检查右侧开关状态，如果是 DOWN，则进入 ZERO_FORCE 模式
        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
        {
            direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ZERO_FORCE;
            direct_gimbal_cmd_ptr->pitch_delta_angle = 0;
            direct_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        }
        direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        direct_gimbal_cmd_ptr->pitch_delta_angle = -p_ctrl->rc.ch_ly * 0.01f;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = p_ctrl->rc.ch_lx * 0.01f;
    }

    void start_app_thread(void *argument)
    {
        // mec_chassis_ptr->start();
        direct_gimbal_ptr->start();
        while (true)
        {
            // chassis_rc2cmd(rc_ctrl_ptr);
            // mec_chassis_ptr->set_command(*mec_cmd_ptr);
            gimbal_rc2cmd(rc_ctrl_ptr);
            direct_gimbal_ptr->set_command(*direct_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void pyro_app_init_thread(void *argument)
    {
        // Initialize Hybrid Chassis
        // mec_chassis_ptr = pyro::mec_chassis_t::instance();
        // mec_cmd_ptr     = new pyro::mec_cmd_t();
        direct_gimbal_ptr = pyro::direct_gimbal_t::instance();
        direct_gimbal_cmd_ptr = new pyro::direct_gimbal_cmd_t();
        rc_ctrl_ptr     = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(start_app_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}