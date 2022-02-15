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
MedicineReminder.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MedicineReminder.list \

OBJCOPY_HEX += \
MedicineReminder.hex \

OBJCOPY_BIN += \
MedicineReminder.bin \


# All Target
all: MedicineReminder.elf secondary-outputs

# Tool invocations
MedicineReminder.elf: $(OBJS) $(USER_OBJS) C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV\ godina\MS\mikroprocesorski-sistemi\Projekat\ -\ Ispit\MedicineReminder\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "MedicineReminder.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\PC-Korisnik\Desktop\NikolaRnjak_80-2018\IV godina\MS\mikroprocesorski-sistemi\Projekat - Ispit\MedicineReminder\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MedicineReminder.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MedicineReminder.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MedicineReminder.list"
	@echo 'Finished building: $@'
	@echo ' '

MedicineReminder.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "MedicineReminder.hex"
	@echo 'Finished building: $@'
	@echo ' '

MedicineReminder.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MedicineReminder.bin"
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
