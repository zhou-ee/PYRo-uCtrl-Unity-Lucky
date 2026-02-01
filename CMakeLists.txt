cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 1. 编译器与工具链配置 (已由 gcc-arm-none-eabi.cmake 处理)
# ==============================================================================
include(cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# ==============================================================================
# 3. 额外性能优化 (保留工具链未包含的指令)
# ==============================================================================
add_compile_options(
        "-mthumb"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
)

# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/Ported_third_lib/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include"
)

add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# ==============================================================================
# 5. 源文件配置 (按分类排序)
# ==============================================================================
add_executable(${CMAKE_PROJECT_NAME}
        # --- Core ---
        PYRo/Core/ETL/map.cpp
        PYRo/Core/Lock/pyro_mutex.cpp
        PYRo/Core/Lock/pyro_rw_lock.cpp
        PYRo/Core/Task/pyro_task.cpp
        PYRo/Core/Memory/pyro_core_dma_heap.c
        PYRo/Core/Memory/pyro_core_mem.cpp

        # --- Peripheral ---
        PYRo/Peripheral/CAN/pyro_can_drv.cpp
        PYRo/Peripheral/DWT/pyro_dwt_drv.cpp
        PYRo/Peripheral/UART/pyro_uart_drv.cpp

        # --- Algorithm ---
        PYRo/Algorithm/Kinematics/pyro_kin_hybrid.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_mec.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_rudder.cpp
        PYRo/Algorithm/OLS/pyro_algo_ols.cpp
        PYRo/Algorithm/PID/pyro_algo_pid.cpp

        # --- Component ---
        PYRo/Component/CRC/PYRo_crc.cpp
        PYRo/Component/Motor/pyro_dji_motor_drv.cpp
        PYRo/Component/Motor/pyro_dm_motor_drv.cpp
        PYRo/Component/Motor/pyro_motor_base.cpp
        PYRo/Component/Powercontrol/pyro_power_control_drv.cpp
        PYRo/Component/Powermeter/pyro_powermeter.cpp
        PYRo/Component/RC/pyro_dr16_rc_drv.cpp
        PYRo/Component/RC/pyro_rc_base_drv.cpp
        PYRo/Component/RC/pyro_rc_hub.cpp
        PYRo/Component/RC/pyro_vt03_rc_drv.cpp
        PYRo/Component/Referee/CRC8_CRC16.c
        PYRo/Component/Referee/fifo.c
        PYRo/Component/Referee/referee.c
        PYRo/Component/Referee/referee_drv.cpp
        PYRo/Component/Referee/referee_usart_task.c

        # --- Module ---
        PYRo/Module/Chassis/Hybrid/pyro_hybrid_chassis.cpp
        PYRo/Module/pyro_module_base.cpp

        # --- Application ---
        PYRo/Application/Demo/demo_task.cpp
        PYRo/Application/Demo/pyro_control_demo.cpp
        PYRo/Application/Demo/pyro_controller_demo.cpp
        PYRo/Application/Demo/pyro_motor_demo.cpp
        PYRo/Application/Demo/pyro_rc_demo.cpp
        PYRo/Application/Demo/pyro_shoot_demo.cpp
        PYRo/Application/Mission/pyro_init_thread.cpp
        PYRo/Application/Mission/pyro_mission_planer.cpp

        # --- Debug ---
        PYRo/Debug/Debug_task.cpp
        PYRo/Debug/JCOM/pyro_jcom.cpp
        PYRo/Debug/VOFA/pyro_vofa.cpp
        PYRo/Module/Chassis/Mecanum/pyro_mec_chassis.cpp
        PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_passive_state.cpp
        PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_active_state.cpp
        PYRo/Application/Mission/pyro_app_thread.cpp
        PYRo/Module/Gimbal/Hero/Direct/pyro_direct_gimbal.cpp
        PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_active_state.cpp
        PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_passive_state.cpp
        PYRo/Communication/pyro_com_cantx.cpp
        PYRo/Communication/pyro_com_canrx.cpp
        PYRo/Module/Booster/Quad/pyro_quad_booster.cpp
        PYRo/Algorithm/Common/pyro_common.cpp
        PYRo/Algorithm/EKF/QuaternionEKF.cpp
        PYRo/Algorithm/KF/kalman_filter.cpp
        PYRo/Component/INS/pyro_ins.cpp
        PYRo/Device/BMI088/BMI088_driver.cpp
        PYRo/Device/BMI088/BMI088_middleware.cpp
)

add_subdirectory(cmake/stm32cubemx)

# ==============================================================================
# 6. 包含路径与链接 (按路径排序)
# ==============================================================================
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        PYRo/Algorithm/Kinematics
        PYRo/Algorithm/OLS
        PYRo/Algorithm/PID
        PYRo/Component/CRC
        PYRo/Component/IMU
        PYRo/Component/Motor
        PYRo/Component/Powercontrol
        PYRo/Component/Powermeter
        PYRo/Component/RC
        PYRo/Component/Referee
        PYRo/Component/Shoot
        PYRo/Core/Config
        PYRo/Core/Def
        PYRo/Core/ETL
        PYRo/Core/FSM
        PYRo/Core/Lock
        PYRo/Core/Memory
        PYRo/Core/Task
        PYRo/Debug/VOFA
        PYRo/Module
        PYRo/Module/Chassis
        PYRo/Module/Chassis/Hybrid
        PYRo/Module/Chassis/Mecanum
        PYRo/Module/Chassis/Rudder
        PYRo/Module/Gimbal/Hero/Direct
        PYRo/Module/Booster/Quad
        PYRo/Peripheral/CAN
        PYRo/Peripheral/DWT
        PYRo/Peripheral/UART
        PYRo/Communication
        PYRo/Algorithm/EKF
        PYRo/Algorithm/KF
        PYRo/Component/INS
        PYRo/Device/BMI088
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        CMSISDSP
        dsppp_lib
)