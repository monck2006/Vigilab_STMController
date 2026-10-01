# C1 运动控制 - GCC 命令行工程（arm-none-eabi-gcc）
# 用法：
#   make            编译，产出 build/C1.elf / C1.hex / C1.bin
#   make clean      清理
# 如果工具链不在默认位置，指定一下：make GCC_PATH=/d/gcc-arm

TARGET   := C1
BUILD    := build
GCC_PATH ?= /d/gcc-arm/bin
PREFIX   := arm-none-eabi-

CC  := $(GCC_PATH)/$(PREFIX)gcc
AS  := $(CC) -x assembler-with-cpp
CP  := $(GCC_PATH)/$(PREFIX)objcopy
SZ  := $(GCC_PATH)/$(PREFIX)size

KEIL := ../C1_Keil

INCLUDES := \
 -I$(KEIL)/Core/Inc \
 -I$(KEIL)/Drivers/STM32F4xx_HAL_Driver/inc \
 -I$(KEIL)/Drivers/CMSIS \
 -I$(KEIL)/Middlewares/FreeRTOS/include \
 -I$(KEIL)/Middlewares/FreeRTOS/portable/GCC/ARM_CM4F \
 -I$(KEIL)/Middlewares/FreeRTOS

C_SOURCES := \
 $(KEIL)/Core/Src/main.c \
 $(KEIL)/Core/Src/stm32f4xx_it.c \
 $(KEIL)/Core/Src/stm32f4xx_hal_msp.c \
 $(KEIL)/Core/Src/protocol.c \
 $(KEIL)/Core/Src/motor.c \
 $(KEIL)/Core/Src/encoder.c \
 $(KEIL)/Core/Src/app_tasks.c \
 $(KEIL)/Drivers/CMSIS/system_stm32f4xx.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_cortex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_rcc.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_rcc_ex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_gpio.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_dma.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_dma_ex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_tim.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_tim_ex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_uart.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_iwdg.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_pwr.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_pwr_ex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_flash.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_flash_ex.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_flash_ramfunc.c \
 $(KEIL)/Drivers/STM32F4xx_HAL_Driver/src/stm32f4xx_hal_exti.c \
 $(KEIL)/Middlewares/FreeRTOS/tasks.c \
 $(KEIL)/Middlewares/FreeRTOS/queue.c \
 $(KEIL)/Middlewares/FreeRTOS/list.c \
 $(KEIL)/Middlewares/FreeRTOS/timers.c \
 $(KEIL)/Middlewares/FreeRTOS/portable/GCC/ARM_CM4F/port.c \
 $(KEIL)/Middlewares/FreeRTOS/portable/MemMang/heap_4.c

ASM_SOURCES := startup_stm32f407xx_gcc.s

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS     := -DUSE_HAL_DRIVER -DSTM32F407xx
CFLAGS   := $(CPUFLAGS) $(DEFS) $(INCLUDES) -std=gnu11 -O2 -g3 \
            -ffunction-sections -fdata-sections -Wall -Wno-unused-parameter \
            -Wno-invalid-utf8 -Wno-unsafe-buffer-usage
ASFLAGS  := $(CPUFLAGS) $(DEFS) -g3

LDSCRIPT := STM32F407ZGTx_FLASH.ld
LIBS     := -lc -lm -lnosys
LDFLAGS  := $(CPUFLAGS) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
            -Wl,-Map=$(BUILD)/$(TARGET).map,--cref -Wl,--gc-sections \
            -Wl,--print-memory-usage

OBJS := $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
OBJS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s .

.PHONY: all clean size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(CP) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(CP) -O binary -S $< $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
