################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
4x4_matrix_keypad.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
4x4_matrix_keypad.list \

OBJCOPY_HEX += \
4x4_matrix_keypad.hex \

OBJCOPY_BIN += \
4x4_matrix_keypad.bin \


# All Target
all: 4x4_matrix_keypad.elf secondary-outputs

# Tool invocations
4x4_matrix_keypad.elf: $(OBJS) $(USER_OBJS) C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV\ godina\MS\mikroprocesorski-sistemi\Domaci_4\stm32\4x4_matrix_keypad\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "4x4_matrix_keypad.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV godina\MS\mikroprocesorski-sistemi\Domaci_4\stm32\4x4_matrix_keypad\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="4x4_matrix_keypad.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

4x4_matrix_keypad.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "4x4_matrix_keypad.list"
	@echo 'Finished building: $@'
	@echo ' '

4x4_matrix_keypad.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "4x4_matrix_keypad.hex"
	@echo 'Finished building: $@'
	@echo ' '

4x4_matrix_keypad.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "4x4_matrix_keypad.bin"
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
