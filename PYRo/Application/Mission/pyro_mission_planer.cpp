#include "cmsis_os.h"
#include "pyro_core_config.h"
extern "C"
{
    extern void pyro_init_thread(void *argument);

#if ROBOT_ID == HERO_ID
#if BOARD_ID == GIMBAL_ID
    extern void hero_gimbal_init(void *argument);
    extern void hero_booster_init(void *argument);
#endif
#endif

    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

#if ROBOT_ID == HERO_ID
#if BOARD_ID == GIMBAL_ID
        xTaskCreate(hero_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#endif
#endif

        vTaskDelete(nullptr);
    }
}