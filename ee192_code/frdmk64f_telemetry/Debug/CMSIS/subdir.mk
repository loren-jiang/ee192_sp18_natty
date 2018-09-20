################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/system_MK64F12.c 

OBJS += \
./CMSIS/system_MK64F12.o 

C_DEPS += \
./CMSIS/system_MK64F12.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/%.o: ../CMSIS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DPRINTF_FLOAT_ENABLE=0 -DFRDM_K64F -DCR_INTEGER_PRINTF -DFREEDOM -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\board" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\source" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\drivers" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\CMSIS" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\utilities" -I"C:\Users\ljiang\Google Drive\Spring_18\ee192\Sp18_team8\frdmk64f_telemetry\startup" -O0 -fno-common -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


