#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_direct_gimbal.h"
#include "pyro_com_cantx.h"

pyro::mec_chassis_t *mec_chassis_ptr             = nullptr;
pyro::mec_cmd_t *mec_cmd_ptr                     = nullptr;
pyro::direct_gimbal_t *direct_gimbal_ptr         = nullptr;
pyro::direct_gimbal_cmd_t *direct_gimbal_cmd_ptr = nullptr;
pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

extern "C"
{
    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
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

    void chassis_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        static int8_t vx      = 0;
        static int8_t vy      = 0;
        static int8_t wz      = 0;
        static uint8_t active = 0;

        pyro::can_tx_drv_t::clear(0x101);
        // 1. 检查左侧开关状态，如果是 DOWN，则进入 ZERO_FORCE 模式
        if (pyro::dr16_drv_t::sw_state_t::SW_DOWN != p_ctrl->rc.s_l.state)
        {
            vx     = 0;
            vy     = 0;
            wz     = 0;
            active = 0;
            pyro::can_tx_drv_t::add_data(0x101, 8, vx);
            pyro::can_tx_drv_t::add_data(0x101, 8, vy);
            pyro::can_tx_drv_t::add_data(0x101, 8, wz);
            pyro::can_tx_drv_t::add_data(0x101, 1, active);
            pyro::can_tx_drv_t::send(
                0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                           pyro::can_hub_t::which_can::can3));
            return;
        }
        vx     = static_cast<int8_t>(p_ctrl->rc.ch_ly * 127);
        vy     = static_cast<int8_t>(-p_ctrl->rc.ch_lx * 127);
        wz     = static_cast<int8_t>(-p_ctrl->rc.ch_rx * 127);
        active = 1;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
    }

    void start_app_thread(void *argument)
    {
        // mec_chassis_ptr->start();
        direct_gimbal_ptr->start();
        while (true)
        {
            chassis_rc2cmd(rc_ctrl_ptr);
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
        direct_gimbal_ptr     = pyro::direct_gimbal_t::instance();
        direct_gimbal_cmd_ptr = new pyro::direct_gimbal_cmd_t();
        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(start_app_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}