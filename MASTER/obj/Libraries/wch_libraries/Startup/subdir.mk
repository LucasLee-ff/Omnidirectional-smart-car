################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Libraries/wch_libraries/Startup/startup_ch32v10x.S 

OBJS += \
./Libraries/wch_libraries/Startup/startup_ch32v10x.o 

S_UPPER_DEPS += \
./Libraries/wch_libraries/Startup/startup_ch32v10x.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/wch_libraries/Startup/%.o: ../Libraries/wch_libraries/Startup/%.S
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -x assembler -I"../Libraries/wch_libraries/Startup" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

