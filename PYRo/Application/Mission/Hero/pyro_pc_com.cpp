#include "pyro_core_dma_heap.h"
#include "pyro_uart_comm.h"
#include "pyro_direct_gimbal.h"
#include "pyro_quad_booster.h"
#include "struct.h"
#include "pyro_crc.h"

using namespace pyro;

pyro::uart_comm_t *uart_comm_ptr = nullptr;

extern pyro::direct_gimbal_t *direct_gimbal_ptr;
extern pyro::quad_booster_t *quad_booster_ptr;

OperateBytes *operate_bytes = nullptr;
StateBytes *state_bytes = nullptr;

extern "C"
{
    void hero_pc_com_thread(void *argument)
    {
        while (true)
        {
            uart_comm_ptr->read(state_bytes, sizeof(StateBytes));
            operate_bytes->frame_header.sof = 0xA5;
            // uart_drv_t::get_instance(uart_drv_t::which_uart::uart10)->write((uint8_t*)operate_bytes, sizeof(OperateBytes),100);
            vTaskDelay(1);
        }
    }

    void hero_pc_com_init(void *argument)
    {
        uart_comm_ptr =
            new uart_comm_t(uart_drv_t::which_uart::uart7, 0x10, 256);
        uint8_t sof = 0xA5;
        uart_comm_ptr->register_msg_type(sizeof(StateBytes), &sof, 1);
        operate_bytes = new OperateBytes();
        operate_bytes->frame_header.sof = sof;
        append_crc16_check_sum((uint8_t*)operate_bytes,sizeof(OperateBytes));
        state_bytes = new StateBytes();
        state_bytes = {};
        xTaskCreate(hero_pc_com_thread, "start_hero_pc_com_thread", 128,

                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}