#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_screw_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

using namespace pyro;

static pyro::screw_gimbal_t *screw_gimbal_ptr           = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr   = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
static pyro::screw_gimbal_deps_t *screw_gimbal_deps     = nullptr;

static void gimbal_rc2cmd(void const *rc_ctrl);
static void chassis_rc2cmd(void const *rc_ctrl);
static void deps_init();

extern "C"
{
    void hero_gimbal_thread(void *argument)
    {
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
        deps_init();
        screw_gimbal_ptr->configure(*screw_gimbal_deps);
        screw_gimbal_ptr->start();
        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}

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
    screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    // screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
    // screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
    screw_gimbal_cmd_ptr->pitch_delta_angle = -p_ctrl->rc.ch_ry * 0.0035f;
    screw_gimbal_cmd_ptr->yaw_delta_angle   = -p_ctrl->rc.ch_rx * 0.002f;
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
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}

void deps_init()
{
    screw_gimbal_deps = new pyro::screw_gimbal_deps_t();

    // 1. 初始化电机

    // Pitch: 使用 DM 电机 (示例 ID: Master 0x11, Slave 0x21, CAN1)
    // 根据 hybrid 中的用法进行配置
    screw_gimbal_deps->motor_deps.pitch = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                    can_hub_t::can3);

    // Yaw: 使用 DJI GM6020 (ID 2, CAN1)
   screw_gimbal_deps->motor_deps.yaw   = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                   can_hub_t::can3);
    

    // 3. 初始化串级 PID
    screw_gimbal_deps->pid_deps.pitch_pos =
        new pid_t(10.0f, 0.1f, 3.4f, 2.0f, 25.0f, 20, 10,
                  4); // 位置环输出为 rad/s，限制在电机可接受范围内
    screw_gimbal_deps->pid_deps.pitch_spd = new pid_t(1.4f, 0.02f, 0.025f, 1.5f, 22.0f, 40, 20,
                                   4); // 输出限制匹配 DM 电机 Nm 级

    // Yaw 轴 (DJI GM6020，输出为电流值/电压值，通常量级较大，如 +/- 30000)
    screw_gimbal_deps->pid_deps.yaw_pos   = new pid_t(5.2f, 0.01f, 0.22f, 0.8f, 5.0f);
    screw_gimbal_deps->pid_deps.yaw_spd   = new pid_t(3.0f, 0.0003f, 0.0001f, 0.2f, 3.0f);
    
}