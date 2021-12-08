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
Nikola_Rnjak_80-2018.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Nikola_Rnjak_80-2018.list \

OBJCOPY_HEX += \
Nikola_Rnjak_80-2018.hex \

OBJCOPY_BIN += \
Nikola_Rnjak_80-2018.bin \


# All Target
all: Nikola_Rnjak_80-2018.elf secondary-outputs

# Tool invocations
Nikola_Rnjak_80-2018.elf: $(OBJS) $(USER_OBJS) C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV\ godina\MS\mikroprocesorski-sistemi\Nikola_Rnjak_80-2018\STM32IDE\Nikola_Rnjak_80-2018\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "Nikola_Rnjak_80-2018.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV godina\MS\mikroprocesorski-sistemi\Nikola_Rnjak_80-2018\STM32IDE\Nikola_Rnjak_80-2018\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Nikola_Rnjak_80-2018.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Nikola_Rnjak_80-2018.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Nikola_Rnjak_80-2018.list"
	@echo 'Finished building: $@'
	@echo ' '

Nikola_Rnjak_80-2018.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Nikola_Rnjak_80-2018.hex"
	@echo 'Finished building: $@'
	@echo ' '

Nikola_Rnjak_80-2018.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Nikola_Rnjak_80-2018.bin"
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
