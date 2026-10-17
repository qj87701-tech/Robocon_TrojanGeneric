cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Robocon-Elian)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    User_File/1_Middleware/Algorithm/Basic/alg_basic.cpp
    User_File/1_Middleware/Algorithm/PID/alg_pid.cpp
    User_File/1_Middleware/Driver/CAN/drv_can.cpp
    User_File/1_Middleware/Driver/UART/drv_uart.cpp
    User_File/1_Middleware/Hipnuc/hipnuc_dec.c
    User_File/1_Middleware/Hipnuc/hipnuc_sample.c
    User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp
    User_File/1_Middleware/Protocol/pro_upper/pro_upper.cpp
    User_File/1_Middleware/System/Timestamp/sys_timestamp.cpp
    User_File/2_Device/BSP/Key/bsp_key.cpp
    User_File/2_Device/IMU/HI06/dvc_imu_hi06.cpp
    User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp
    User_File/2_Device/Motor/Motor_VESC/dvc_motor_vesc.cpp
    User_File/2_Device/NRF24L01/dvc_nrf24l01.cpp
    User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp
    User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp
    User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp
    User_File/4_Task/tsk_motor_init/motor_init.cpp
    User_File/4_Task/tsk_4swerve_chassis/tsk_chassis.cpp
    User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp
    User_File/4_Task/tsk_debug/tsk_debug.cpp
    User_File/4_Task/tsk_remote/tsk_remote.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User_File
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    "${CMAKE_CURRENT_SOURCE_DIR}/User_File/1_Middleware/Library/CMSIS_DSP/libarm_cortexM7lfsp_math.a"
)
