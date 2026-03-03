#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_screw_gimbal.h"
#include "pyro_com_cantx.h"


static pyro::screw_gimbal_t *screw_gimbal_ptr         = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

extern "C"
{
    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
        {
            screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
            screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
            return;
        }
        screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        // screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
        // screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        screw_gimbal_cmd_ptr->pitch_delta_angle = -p_ctrl->rc.ch_ry * 0.0035f;
        screw_gimbal_cmd_ptr->yaw_delta_angle   = -p_ctrl->rc.ch_rx * 0.0035f;
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

        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
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
        wz     = 0;
        active = 1;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
    }

    void hero_gimbal_thread(void *argument)
    {
        screw_gimbal_ptr->start();
        while (true)
        {
            chassis_rc2cmd(rc_ctrl_ptr);
            gimbal_rc2cmd(rc_ctrl_ptr);
            screw_gimbal_ptr->set_command(*screw_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_gimbal_init(void *argument)
    {
        screw_gimbal_cmd_ptr = new pyro::screw_gimbal_cmd_t();
        screw_gimbal_ptr     = pyro::screw_gimbal_t::instance();
        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}
