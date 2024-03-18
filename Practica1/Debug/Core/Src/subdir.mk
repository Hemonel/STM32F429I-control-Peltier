################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Consolas9x18Amarillo.c \
../Core/Src/SansSerif8x13Amarillo.c \
../Core/Src/botonReset60x60.c \
../Core/Src/freertos.c \
../Core/Src/ili9341.c \
../Core/Src/imagenAlarma11_70x70.c \
../Core/Src/imagenAlerta1_70x70.c \
../Core/Src/imagenBotonNo50x50.c \
../Core/Src/imagenBotonSi50x50.c \
../Core/Src/imagenCircuito240x320.c \
../Core/Src/imagenMaletin50x50.c \
../Core/Src/imagenMicroscopio50x50.c \
../Core/Src/industria3_240x320.c \
../Core/Src/juegoCaracteres11x16.c \
../Core/Src/juegoCaracteres8x11.c \
../Core/Src/main.c \
../Core/Src/pantalla.c \
../Core/Src/stm32f429i_discovery.c \
../Core/Src/stm32f429i_discovery_io.c \
../Core/Src/stm32f429i_discovery_lcd.c \
../Core/Src/stm32f429i_discovery_sdram.c \
../Core/Src/stm32f429i_discovery_ts.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stmpe811.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Consolas9x18Amarillo.o \
./Core/Src/SansSerif8x13Amarillo.o \
./Core/Src/botonReset60x60.o \
./Core/Src/freertos.o \
./Core/Src/ili9341.o \
./Core/Src/imagenAlarma11_70x70.o \
./Core/Src/imagenAlerta1_70x70.o \
./Core/Src/imagenBotonNo50x50.o \
./Core/Src/imagenBotonSi50x50.o \
./Core/Src/imagenCircuito240x320.o \
./Core/Src/imagenMaletin50x50.o \
./Core/Src/imagenMicroscopio50x50.o \
./Core/Src/industria3_240x320.o \
./Core/Src/juegoCaracteres11x16.o \
./Core/Src/juegoCaracteres8x11.o \
./Core/Src/main.o \
./Core/Src/pantalla.o \
./Core/Src/stm32f429i_discovery.o \
./Core/Src/stm32f429i_discovery_io.o \
./Core/Src/stm32f429i_discovery_lcd.o \
./Core/Src/stm32f429i_discovery_sdram.o \
./Core/Src/stm32f429i_discovery_ts.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/stmpe811.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Consolas9x18Amarillo.d \
./Core/Src/SansSerif8x13Amarillo.d \
./Core/Src/botonReset60x60.d \
./Core/Src/freertos.d \
./Core/Src/ili9341.d \
./Core/Src/imagenAlarma11_70x70.d \
./Core/Src/imagenAlerta1_70x70.d \
./Core/Src/imagenBotonNo50x50.d \
./Core/Src/imagenBotonSi50x50.d \
./Core/Src/imagenCircuito240x320.d \
./Core/Src/imagenMaletin50x50.d \
./Core/Src/imagenMicroscopio50x50.d \
./Core/Src/industria3_240x320.d \
./Core/Src/juegoCaracteres11x16.d \
./Core/Src/juegoCaracteres8x11.d \
./Core/Src/main.d \
./Core/Src/pantalla.d \
./Core/Src/stm32f429i_discovery.d \
./Core/Src/stm32f429i_discovery_io.d \
./Core/Src/stm32f429i_discovery_lcd.d \
./Core/Src/stm32f429i_discovery_sdram.d \
./Core/Src/stm32f429i_discovery_ts.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/stmpe811.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Consolas9x18Amarillo.d ./Core/Src/Consolas9x18Amarillo.o ./Core/Src/Consolas9x18Amarillo.su ./Core/Src/SansSerif8x13Amarillo.d ./Core/Src/SansSerif8x13Amarillo.o ./Core/Src/SansSerif8x13Amarillo.su ./Core/Src/botonReset60x60.d ./Core/Src/botonReset60x60.o ./Core/Src/botonReset60x60.su ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/imagenAlarma11_70x70.d ./Core/Src/imagenAlarma11_70x70.o ./Core/Src/imagenAlarma11_70x70.su ./Core/Src/imagenAlerta1_70x70.d ./Core/Src/imagenAlerta1_70x70.o ./Core/Src/imagenAlerta1_70x70.su ./Core/Src/imagenBotonNo50x50.d ./Core/Src/imagenBotonNo50x50.o ./Core/Src/imagenBotonNo50x50.su ./Core/Src/imagenBotonSi50x50.d ./Core/Src/imagenBotonSi50x50.o ./Core/Src/imagenBotonSi50x50.su ./Core/Src/imagenCircuito240x320.d ./Core/Src/imagenCircuito240x320.o ./Core/Src/imagenCircuito240x320.su ./Core/Src/imagenMaletin50x50.d ./Core/Src/imagenMaletin50x50.o ./Core/Src/imagenMaletin50x50.su ./Core/Src/imagenMicroscopio50x50.d ./Core/Src/imagenMicroscopio50x50.o ./Core/Src/imagenMicroscopio50x50.su ./Core/Src/industria3_240x320.d ./Core/Src/industria3_240x320.o ./Core/Src/industria3_240x320.su ./Core/Src/juegoCaracteres11x16.d ./Core/Src/juegoCaracteres11x16.o ./Core/Src/juegoCaracteres11x16.su ./Core/Src/juegoCaracteres8x11.d ./Core/Src/juegoCaracteres8x11.o ./Core/Src/juegoCaracteres8x11.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pantalla.d ./Core/Src/pantalla.o ./Core/Src/pantalla.su ./Core/Src/stm32f429i_discovery.d ./Core/Src/stm32f429i_discovery.o ./Core/Src/stm32f429i_discovery.su ./Core/Src/stm32f429i_discovery_io.d ./Core/Src/stm32f429i_discovery_io.o ./Core/Src/stm32f429i_discovery_io.su ./Core/Src/stm32f429i_discovery_lcd.d ./Core/Src/stm32f429i_discovery_lcd.o ./Core/Src/stm32f429i_discovery_lcd.su ./Core/Src/stm32f429i_discovery_sdram.d ./Core/Src/stm32f429i_discovery_sdram.o ./Core/Src/stm32f429i_discovery_sdram.su ./Core/Src/stm32f429i_discovery_ts.d ./Core/Src/stm32f429i_discovery_ts.o ./Core/Src/stm32f429i_discovery_ts.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/stmpe811.d ./Core/Src/stmpe811.o ./Core/Src/stmpe811.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

