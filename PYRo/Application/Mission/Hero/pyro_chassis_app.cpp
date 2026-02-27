#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_canrx.h"

static pyro::mec_chassis_t *mec_chassis_ptr             = nullptr;
static pyro::mec_cmd_t *mec_cmd_ptr                     = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *dr16_ctrl_ptr = nullptr;

extern "C"
{
    void chassis_rxcmd(void const *rc_ctrl)
    {
        std::array<uint8_t, 8> raw_data{};
        pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can2, 0x101,
                                     raw_data);
        mec_cmd_ptr->vx     = 3 * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
        if (abs(mec_cmd_ptr->vx) < 0.1f)
        {
            mec_cmd_ptr->vx = 0;
        }
        mec_cmd_ptr->vy     = 3 * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
        if (abs(mec_cmd_ptr->vy) < 0.1f)
        {
            mec_cmd_ptr->vy = 0;
        }
        mec_cmd_ptr->wz     = 3 * static_cast<float>(static_cast<int8_t>(raw_data[2])) / 127.0f;
        if (abs(mec_cmd_ptr->wz) < 0.1f)
        {
            mec_cmd_ptr->wz = 0;
        }
        mec_cmd_ptr->mode = static_cast<pyro::cmd_base_t::mode_t>(raw_data[3]);
    }

    void chassis_dr162cmd(pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());

        if (pyro::dr16_drv_t::sw_state_t::SW_MID != rc_ctrl->rc.s_r.state)
        {
            mec_cmd_ptr->vx   = 0;
            mec_cmd_ptr->vy   = 0;
            mec_cmd_ptr->wz   = 0;
            mec_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        }
        else
        {
            mec_cmd_ptr->vx =
                3 * static_cast<float>(rc_ctrl->rc.ch_ly) / 127.0f;
            mec_cmd_ptr->vy =
                3 * static_cast<float>(-rc_ctrl->rc.ch_lx) / 127.0f;
            mec_cmd_ptr->wz   = 0;
            mec_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        }
    }

    void hero_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rxcmd(dr16_ctrl_ptr);
            mec_chassis_ptr->set_command(*mec_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can2, 0x101);
        mec_cmd_ptr     = new pyro::mec_cmd_t();
        mec_chassis_ptr = pyro::mec_chassis_t::instance();
        dr16_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        mec_chassis_ptr->start();
        xTaskCreate(hero_chassis_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}