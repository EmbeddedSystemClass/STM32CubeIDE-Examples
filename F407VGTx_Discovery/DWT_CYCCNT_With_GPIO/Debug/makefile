################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
DWT_CYCCNT_With_GPIO.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DWT_CYCCNT_With_GPIO.list \


# All Target
all: DWT_CYCCNT_With_GPIO.elf secondary-outputs

# Tool invocations
DWT_CYCCNT_With_GPIO.elf: $(OBJS) $(USER_OBJS) /home/jerryl/STM32CubeIDE/stm32cubeide_samples/F407VGTx_Discovery/DWT_CYCCNT_With_GPIO/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "DWT_CYCCNT_With_GPIO.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/jerryl/STM32CubeIDE/stm32cubeide_samples/F407VGTx_Discovery/DWT_CYCCNT_With_GPIO/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DWT_CYCCNT_With_GPIO.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DWT_CYCCNT_With_GPIO.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DWT_CYCCNT_With_GPIO.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
