################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
audiometer1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
audiometer1.list \

OBJCOPY_HEX += \
audiometer1.hex \

OBJCOPY_BIN += \
audiometer1.bin \


# All Target
all: audiometer1.elf secondary-outputs

# Tool invocations
audiometer1.elf: $(OBJS) $(USER_OBJS) /home/ali/STM32CubeIDE/workspace_1.3.0/audiometer1/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "audiometer1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/ali/STM32CubeIDE/workspace_1.3.0/audiometer1/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="audiometer1.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

audiometer1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "audiometer1.list"
	@echo 'Finished building: $@'
	@echo ' '

audiometer1.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "audiometer1.hex"
	@echo 'Finished building: $@'
	@echo ' '

audiometer1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "audiometer1.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
