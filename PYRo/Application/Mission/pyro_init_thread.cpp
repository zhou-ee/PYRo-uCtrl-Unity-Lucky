#include "pyro_can_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"

using namespace pyro;
extern "C" void pyro_chassis(void* argument);
float angle[3];
float gyro[3];
extern "C"
{
    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;

    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz
        
        uart_drv_t::get_instance(uart_drv_t::uart1)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::uart5)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::uart7)
            ->enable_rx_dma();
        uart_drv_t::get_instance(uart_drv_t::uart10)
            ->enable_rx_dma();

        rc_hub_t::get_instance(rc_hub_t::DR16)->enable();
        rc_hub_t::get_instance(rc_hub_t::VT03)->enable();

        can_hub_t::get_instance();
        can1_drv = new can_drv_t(&hfdcan1);
        can2_drv = new can_drv_t(&hfdcan2);
        can3_drv = new can_drv_t(&hfdcan3);
        can1_drv->init();
        can2_drv->init();
        can3_drv->init();
        can1_drv->start();
        can2_drv->start();
        can3_drv->start();

        ins_drv = ins_drv_t::get_instance();
        ins_drv->init();
        while(1)
        {
            ins_drv->get_angles_n(&angle[0], &angle[1], &angle[2]);
            ins_drv->get_gyro_b(&gyro[0], &gyro[1], &gyro[2]);
            vTaskDelay(1);
        }
        // xTaskCreate(pyro_chassis, "chassis", 256, nullptr, 1, nullptr);
        // vTaskDelete(nullptr);
    }
}