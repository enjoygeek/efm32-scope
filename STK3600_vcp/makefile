RM := rm -rf
SDK_PATH = ../sdk
CC := arm-none-eabi-gcc
EXECUTABLE := vcp

S_SRC := $(SDK_PATH)/Device/SiliconLabs/EFM32LG/Source/GCC/startup_gcc_efm32lg.s

C_SRC := ./$(SDK_PATH)/Device/SiliconLabs/EFM32LG/Source/system_efm32lg.c \

C_SRC += $(SDK_PATH)/emlib/src/em_int.c \
		 $(SDK_PATH)/emlib/src/em_cmu.c \
		 $(SDK_PATH)/emlib/src/em_emu.c \
		 $(SDK_PATH)/emlib/src/em_gpio.c \
		 $(SDK_PATH)/emlib/src/em_timer.c \
		 $(SDK_PATH)/emlib/src/em_prs.c \
		 $(SDK_PATH)/emlib/src/em_lcd.c \
		 $(SDK_PATH)/emlib/src/em_system.c \
		 $(SDK_PATH)/emlib/src/em_rmu.c \
		 $(SDK_PATH)/emlib/src/em_rtc.c \
		 $(SDK_PATH)/emlib/src/em_adc.c \
		 $(SDK_PATH)/emlib/src/em_dac.c \
		 $(SDK_PATH)/emlib/src/em_dma.c \
		 $(SDK_PATH)/emdrv/sleep/src/sleep.c \
		 $(SDK_PATH)/emdrv/dmadrv/src/dmadrv.c \

C_SRC += $(SDK_PATH)/usb/src/em_usbd.c \
		 $(SDK_PATH)/usb/src/em_usbdint.c \
		 $(SDK_PATH)/usb/src/em_usbhal.c \
		 $(SDK_PATH)/usb/src/em_usbdep.c \
		 $(SDK_PATH)/usb/src/em_usbdch9.c \
		 $(SDK_PATH)/usb/src/em_usbtimer.c \

C_SRC += ./BSP/bsp_trace.c \
		 ./BSP/bsp_stk_leds.c

C_SRC += ./src/main.c \
		 ./src/retargetio.c \
		 ./src/descriptors.c \
		 ./src/low_power_tick_management.c \
		 ./src/adcTask.c \
		 ./Drivers/cdc.c \
		 ./Drivers/segmentlcd.c \
		 ./Drivers/dmactrl.c \

FreeRTOS_Heapmode = 1
C_SRC += $(wildcard $(SDK_PATH)/FreeRTOS/Source/*.c) \
		 $(SDK_PATH)/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
		 $(SDK_PATH)/FreeRTOS/Source/portable/MemMang/heap_$(FreeRTOS_Heapmode).c

C_OBJS := $(C_SRC:.c=.o)
S_OBJS := $(S_SRC:.s=.a)
OBJS = $(C_OBJS) $(S_OBJS)

INCLUDE_DIRS = -I./src \
			   -I$(SDK_PATH)/CMSIS/Include \
			   -I$(SDK_PATH)/emlib/inc \
			   -I$(SDK_PATH)/emdrv/common/inc \
			   -I$(SDK_PATH)/emdrv/config \
			   -I$(SDK_PATH)/emdrv/sleep/inc \
			   -I$(SDK_PATH)/emdrv/dmadrv/inc \
			   -I$(SDK_PATH)/usb/inc \
			   -I$(SDK_PATH)/Device/SiliconLabs/EFM32LG/Include \
			   -I$(SDK_PATH)/kits/common/bsp \
			   -I$(SDK_PATH)/kits/common/drivers \
			   -I$(SDK_PATH)/kits/EFM32LG_STK3600/config \
			   -I$(SDK_PATH)/FreeRTOS/Source/include \
			   -I$(SDK_PATH)/FreeRTOS/Source/portable/GCC/ARM_CM3

#-I$(SDK_PATH)/kits/common/drivers \

CFLAGS := -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DDEBUG_EFM_USER=1' '-DEFM32LG990F256=1' '-DDEBUG=1' -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c11
CFLAGS += $(INCLUDE_DIRS)

%.o: %.c
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@

%.a: %.s
	@echo 'Building object: $(abspath $@)'
	$(CC) $(CFLAGS) $^ -o $@


release: all

# All Target
all: $(EXECUTABLE).axf

# Tool invocations
$(EXECUTABLE).axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -T "STK3600_vcp.ld" -Xlinker --gc-sections -Xlinker -Map="$(EXECUTABLE).map" --specs=nano.specs -o $@ $(OBJS) -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: $(EXECUTABLE).hex'
	arm-none-eabi-objcopy -O ihex "$(EXECUTABLE).axf" "$(EXECUTABLE).hex"
	@echo ' '

	@echo 'Building bin file: $(EXECUTABLE).bin'
	arm-none-eabi-objcopy -O binary "$(EXECUTABLE).axf" "$(EXECUTABLE).bin"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "$(EXECUTABLE).axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(EXECUTABLE).axf $(EXECUTABLE).bin $(EXECUTABLE).hex $(EXECUTABLE).map
	-@echo ' '

upload: $(EXECUTABLE).bin
	@echo 'Uploading $(EXECUTABLE).bin'
	minicom -D /dev/ttyACM0 -S upload.txt

.PHONY: all clean dependents upload
.SECONDARY:
