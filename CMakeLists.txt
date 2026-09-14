cmake_minimum_required(VERSION 3.22)

project(HexaArachnid C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 方便 VS Code/clangd 索引。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 统一输出目录，便于管理产物。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

###############################################################
# 本工程已固定为单一主控 STM32F407（蜘蛛机器人）。
# 不再做多 MCU 目标切换，F407 平台配置直接内联。
###############################################################
set(TARGET_NAME HexaArachnid_F407)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Core/STM32F407/cmake/STM32F407VE_FLASH.ld)
set(OPENOCD_CFG ${CMAKE_SOURCE_DIR}/OpenOCD/F407_OpenOCD.cfg)
set(OPENOCD_ERASE_CMD "stm32f4x mass_erase 0")
set(MCU_CPU_FLAGS -mcpu=cortex-m4 -mthumb)
set(MCU_FPU_FLAGS -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(MCU_DEFINITIONS STM32F40_41xxx)

# ============================== 维护说明 ==============================
#
# 1) APP、API、SYS、ENROLL 层为通用代码，只需维护一份源文件列表（见下方变量）。
#    - 新增/删除 .c 文件，直接在 APP_LAYER_SOURCES、API_LAYER_SOURCES、SYS_LAYER_SOURCES、ENROLL_LAYER_SOURCES 里增删。
#
# 2) BSP 层（BSP_LAYER_SOURCES）：公共驱动 + F407 专属驱动统一维护在一处。
#
# 3) 头文件目录：通用目录放 COMMON_INCLUDE_DIRS / BSP_COMMON_INCLUDE_DIRS，
#    F407 平台目录放 MCU_INCLUDE_DIRS。
#
# 4) MCU_SOURCES 只放 F407 平台专属代码（Core 底层 + 启动文件）。
#
# 5) 改完后验证：
#    - 重新 Configure/Build；若报“undefined reference”通常是漏加 .c 文件。
#    - 若报“header not found”通常是漏加头文件目录到 MCU_INCLUDE_DIRS。
# =====================================================================

# 应用层（app）：业务入口与任务调度。
set(APP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/A_Entry/main.c
    ${CMAKE_SOURCE_DIR}/app/My_Usart/My_Usart.c
    ${CMAKE_SOURCE_DIR}/app/Control_Task/Control_Task.c
    ${CMAKE_SOURCE_DIR}/app/PID/PID.c
    ${CMAKE_SOURCE_DIR}/app/Filter/Filter.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/leg.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/servo.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/movement.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/movement_profile.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/hexapod.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/command.c
)

# 系统层（SYSTEM）：中断管理与板级系统封装。
set(SYS_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/SYSTEM/sys.c
)

# API 层：对上提供统一外设接口，对下对接 F407 Core 实现。
set(API_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/API/src/adc.c
    ${CMAKE_SOURCE_DIR}/API/src/exti.c
    ${CMAKE_SOURCE_DIR}/API/src/gpio.c
    ${CMAKE_SOURCE_DIR}/API/src/tim.c
    ${CMAKE_SOURCE_DIR}/API/src/usart.c
    ${CMAKE_SOURCE_DIR}/API/src/pwm.c
    ${CMAKE_SOURCE_DIR}/API/API_I2C/API_I2C.c
    ${CMAKE_SOURCE_DIR}/API/API_SPI/API_SPI.c
)

# 注册层（Enroll）：把 BSP 资源和具体 MCU 驱动绑定。
set(ENROLL_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/Enroll/Enroll.c
)

# FreeRTOS 内核（V11.1.0，官方 FreeRTOS-Kernel 仓库源码）。
# - 移植层：portable/GCC/ARM_CM4F（Cortex-M4F 硬件浮点）
# - 内存管理：heap_4
# - 配置文件 FreeRTOSConfig.h 位于 RTOS/ 目录
set(RTOS_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/FreeRTOS/tasks.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/queue.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/list.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/timers.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/event_groups.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/MemMang/heap_4.c
)

# FreeRTOS 头文件目录（RTOS/ 存放 FreeRTOSConfig.h）。
set(RTOS_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/RTOS
    ${CMAKE_SOURCE_DIR}/FreeRTOS/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
)

# 通用头文件目录。
set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/A_Entry
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/app/Control_Task
    ${CMAKE_SOURCE_DIR}/app/My_Usart
    ${CMAKE_SOURCE_DIR}/app/PID
    ${CMAKE_SOURCE_DIR}/app/Filter
    ${CMAKE_SOURCE_DIR}/app/Hexa
    ${CMAKE_SOURCE_DIR}/API
    ${CMAKE_SOURCE_DIR}/API/inc
    ${CMAKE_SOURCE_DIR}/API/API_I2C
    ${CMAKE_SOURCE_DIR}/API/API_SPI
    ${CMAKE_SOURCE_DIR}/Enroll
    ${CMAKE_SOURCE_DIR}/SYSTEM
)

# 通用 BSP 头文件目录。
set(BSP_COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/BSP
    ${CMAKE_SOURCE_DIR}/BSP/LED
    ${CMAKE_SOURCE_DIR}/BSP/KEY
    ${CMAKE_SOURCE_DIR}/BSP/OLED
    ${CMAKE_SOURCE_DIR}/BSP/HCSR04
    ${CMAKE_SOURCE_DIR}/BSP/JY61P
    ${CMAKE_SOURCE_DIR}/BSP/SU03T
    ${CMAKE_SOURCE_DIR}/BSP/PCA9685
)

# 板级支持层（BSP）：公共驱动 + F407 专属传感器。
set(BSP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/BSP/LED/LED.c
    ${CMAKE_SOURCE_DIR}/BSP/KEY/KEY.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED_Data.c
    ${CMAKE_SOURCE_DIR}/BSP/HCSR04/HCSR04.c
    ${CMAKE_SOURCE_DIR}/BSP/JY61P/jy61p.c
    ${CMAKE_SOURCE_DIR}/BSP/SU03T/SU03T.c
    ${CMAKE_SOURCE_DIR}/BSP/BMP280/BMP280.c
    ${CMAKE_SOURCE_DIR}/BSP/NRF24L01/NRF24L01.c
    ${CMAKE_SOURCE_DIR}/BSP/QMC5883P/QMC5883P.c
    ${CMAKE_SOURCE_DIR}/BSP/PCA9685/PCA9685.c
)

# F407 头文件目录（通用 + 平台专属）。
set(MCU_INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
    ${BSP_COMMON_INCLUDE_DIRS}
    ${RTOS_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/BSP/BMP280
    ${CMAKE_SOURCE_DIR}/BSP/NRF24L01
    ${CMAKE_SOURCE_DIR}/BSP/QMC5883P
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/inc
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_i2c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_spi
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start
)

# F407 专属 MCU 驱动实现（Core 底层 + 启动文件）。
set(MCU_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_delay.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_i2c/f407_soft_i2c.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_spi/f407_soft_spi.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_exti.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_adc.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_gpio.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_tim.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_usart.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_pwm.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start/startup_stm32f407vetx_gcc.c
)


###############################################################
# 汇总所有层的源文件，形成最终构建目标源文件列表
###############################################################
set(TARGET_SOURCES
    ${APP_LAYER_SOURCES}
    ${SYS_LAYER_SOURCES}
    ${API_LAYER_SOURCES}
    ${ENROLL_LAYER_SOURCES}
    ${BSP_LAYER_SOURCES}
    ${MCU_SOURCES}
    ${RTOS_LAYER_SOURCES}
)


###############################################################
# 构建最终可执行文件（固件 ELF）
###############################################################
add_executable(${TARGET_NAME} ${TARGET_SOURCES})


###############################################################
# 添加头文件搜索路径
###############################################################
target_include_directories(${TARGET_NAME} PRIVATE
    ${MCU_INCLUDE_DIRS}
)


###############################################################
# 添加编译宏定义（平台专属宏）
###############################################################
target_compile_definitions(${TARGET_NAME} PRIVATE
    ${MCU_DEFINITIONS}
)


###############################################################
# 添加编译器选项（CPU/FPU 架构、警告开关等）
###############################################################
target_compile_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -Wall
    -Wextra
)


###############################################################
# 添加链接器选项（链接脚本、浮点格式、内存优化等）
###############################################################
target_link_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nosys.specs
    --specs=nano.specs
    -u
    _printf_float
    -Wl,--gc-sections
    -Wl,-Map=${TARGET_NAME}.map
    -Wl,--print-memory-usage
)


###############################################################
# 链接标准库（m: 数学库, c: 标准C库）
###############################################################
target_link_libraries(${TARGET_NAME} PRIVATE
    m
    c
)


###############################################################
# 生成常用烧录文件格式（hex/bin）及打印固件大小
###############################################################
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generate .hex/.bin and print size"
)


###############################################################
# OpenOCD 烧录/擦除目标
# - flash: 烧录固件
# - erase: 全片擦除
###############################################################
add_custom_target(flash
    COMMAND openocd -f ${OPENOCD_CFG} -c "program $<TARGET_FILE:${TARGET_NAME}> verify reset exit"
    DEPENDS ${TARGET_NAME}
    COMMENT "Flash firmware via OpenOCD"
)

add_custom_target(erase
    COMMAND openocd -f ${OPENOCD_CFG} -c "init; reset halt; ${OPENOCD_ERASE_CMD}; reset run; exit"
    COMMENT "Mass erase firmware via OpenOCD"
)
