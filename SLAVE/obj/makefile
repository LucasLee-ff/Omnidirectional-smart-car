################################################################################
# 自动生成的文件。不要编辑！
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USER/subdir.mk
-include Libraries/wch_libraries/Startup/subdir.mk
-include Libraries/wch_libraries/Peripheral/subdir.mk
-include Libraries/wch_libraries/Core/subdir.mk
-include Libraries/seekfree_peripheral/subdir.mk
-include Libraries/seekfree_libraries/subdir.mk
-include Libraries/board/subdir.mk
-include CODE/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
100_key_SLAVE.hex \

SECONDARY_LIST += \
100_key_SLAVE.lst \

SECONDARY_SIZE += \
100_key_SLAVE.siz \


# 所有目标
all: 100_key_SLAVE.elf secondary-outputs

# 工具调用
100_key_SLAVE.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T .ld -nostartfiles -Xlinker --gc-sections -L"../" -Wl,-Map,"100_key_SLAVE.map" --specs=nano.specs --specs=nosys.specs -o "100_key_SLAVE.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
100_key_SLAVE.hex: 100_key_SLAVE.elf
	@	riscv-none-embed-objcopy -O ihex "100_key_SLAVE.elf"  "100_key_SLAVE.hex"
	@	@
100_key_SLAVE.lst: 100_key_SLAVE.elf
	@	riscv-none-embed-objdump --source --all-headers --demangle --line-numbers --wide "100_key_SLAVE.elf" > "100_key_SLAVE.lst"
	@	@
100_key_SLAVE.siz: 100_key_SLAVE.elf
	@	riscv-none-embed-size --format=berkeley "100_key_SLAVE.elf"
	@	@
# 其他目标
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) 100_key_SLAVE.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
