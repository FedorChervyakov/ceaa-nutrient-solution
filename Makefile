##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.5.2] date: [Sat Oct 19 18:45:49 EEST 2019] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = ceaa-nutrient-solution


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/app_freertos.c \
Core/Src/stm_logging.c \
Core/Src/app_entry.c \
Core/Src/hw_timerserver.c \
Core/Src/stm32_lpm_if.c \
Core/Src/freertos_port.c \
Core/Src/stm32wbxx_it.c \
Core/Src/stm32wbxx_hal_msp.c \
Core/Src/stm32wbxx_hal_timebase_tim.c \
Core/Src/sensors.c \
STM32_WPAN/Target/hw_ipcc.c \
STM32_WPAN/App/app_thread.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_adc.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c \
Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c \
Core/Src/system_stm32wbxx.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c \
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c \
Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c \
Middlewares/ST/STM32_WPAN/utilities/otp.c \
Middlewares/ST/STM32_WPAN/utilities/stm_list.c \
Middlewares/ST/STM32_WPAN/utilities/stm_queue.c \
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_thread_hci.c \
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.c \
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/dns.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/thread.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/radio.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/channel_monitor.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/commissioner.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/server.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/openthread_api_wb.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/dataset_ftd.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/dataset.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/joiner.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/tasklet.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/link_raw.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/thread_ftd.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/link.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/icmp6.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/netdata.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/dhcp6_server.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/crypto.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/channel_manager.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/jam_detection.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/openthread.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/dhcp6_client.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/coap.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/ip6.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/udp.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/child_supervision.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/message.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/diag.c \
Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api/instance.c \
Utilities/lpm/tiny_lpm/stm32_lpm.c \
adafruit-motor-shield-v2-lib/Source/motor_shield.c \
adafruit-motor-shield-v2-lib/Source/pca9685.c

# ASM sources
ASM_SOURCES =  \
startup_stm32wb55xx_cm4.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DOPENTHREAD_CONFIG_FILE=\"openthread_api_config_ftd.h\" \
-DTHREAD_WB \
-DUSE_HAL_DRIVER \
-DSTM32WB55xx 

# AS includes
AS_INCLUDES =  \
-ICore/Inc

# C includes
C_INCLUDES =  \
-ICore/Inc \
-ISTM32_WPAN/App \
-IDrivers/STM32WBxx_HAL_Driver/Inc \
-IUtilities/lpm/tiny_lpm \
-I../tiny_lpm \
-IDrivers/STM32WBxx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IMiddlewares/ST/STM32_WPAN \
-IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread \
-IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl \
-IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci \
-IMiddlewares/ST/STM32_WPAN/utilities \
-IDrivers/CMSIS/Device/ST/STM32WBxx/Include \
-IMiddlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api \
-IMiddlewares/ST/STM32_WPAN/thread/openthread/stack/include \
-IMiddlewares/ST/STM32_WPAN/thread/openthread/stack/include/openthread \
-IMiddlewares/ST/STM32_WPAN/thread/openthread/stack/include/openthread/platform \
-IDrivers/CMSIS/Include \
-Iadafruit-motor-shield-v2-lib/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = stm32wb55xx_flash_cm4.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs \
                 -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
                 -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
