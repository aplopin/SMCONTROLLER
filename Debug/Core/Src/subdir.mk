################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/driver.c \
../Core/Src/dwt.c \
../Core/Src/fifo.c \
../Core/Src/fifo_char.c \
../Core/Src/gcode.c \
../Core/Src/interpolator.c \
../Core/Src/laser.c \
../Core/Src/main.c \
../Core/Src/net.c \
../Core/Src/planner.c \
../Core/Src/stepper.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/driver.o \
./Core/Src/dwt.o \
./Core/Src/fifo.o \
./Core/Src/fifo_char.o \
./Core/Src/gcode.o \
./Core/Src/interpolator.o \
./Core/Src/laser.o \
./Core/Src/main.o \
./Core/Src/net.o \
./Core/Src/planner.o \
./Core/Src/stepper.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/driver.d \
./Core/Src/dwt.d \
./Core/Src/fifo.d \
./Core/Src/fifo_char.d \
./Core/Src/gcode.d \
./Core/Src/interpolator.d \
./Core/Src/laser.d \
./Core/Src/main.d \
./Core/Src/net.d \
./Core/Src/planner.d \
./Core/Src/stepper.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/BSP/Components/lan8742 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/driver.cyclo ./Core/Src/driver.d ./Core/Src/driver.o ./Core/Src/driver.su ./Core/Src/dwt.cyclo ./Core/Src/dwt.d ./Core/Src/dwt.o ./Core/Src/dwt.su ./Core/Src/fifo.cyclo ./Core/Src/fifo.d ./Core/Src/fifo.o ./Core/Src/fifo.su ./Core/Src/fifo_char.cyclo ./Core/Src/fifo_char.d ./Core/Src/fifo_char.o ./Core/Src/fifo_char.su ./Core/Src/gcode.cyclo ./Core/Src/gcode.d ./Core/Src/gcode.o ./Core/Src/gcode.su ./Core/Src/interpolator.cyclo ./Core/Src/interpolator.d ./Core/Src/interpolator.o ./Core/Src/interpolator.su ./Core/Src/laser.cyclo ./Core/Src/laser.d ./Core/Src/laser.o ./Core/Src/laser.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/net.cyclo ./Core/Src/net.d ./Core/Src/net.o ./Core/Src/net.su ./Core/Src/planner.cyclo ./Core/Src/planner.d ./Core/Src/planner.o ./Core/Src/planner.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

