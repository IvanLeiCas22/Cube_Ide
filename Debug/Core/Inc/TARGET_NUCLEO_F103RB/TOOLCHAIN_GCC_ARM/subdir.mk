################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/PeripheralPins.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/analogin_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/can_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/cmsis_nvic.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/gpio_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/gpio_irq_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/hal_tick_16b.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/hal_tick_32b.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/i2c_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/lp_ticker.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/mbed_board.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/mbed_overrides.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/pinmap.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/port_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/pwmout_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/retarget.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/rtc_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/serial_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/sleep.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/spi_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/startup_stm32f103xb.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_adc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_adc_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_can.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_cec.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_cortex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_crc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dac.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dac_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dma.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_eth.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_flash.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_flash_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_gpio.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_gpio_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_hcd.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_i2c.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_i2s.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_irda.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_iwdg.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_nand.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_nor.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pccard.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pcd.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pcd_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pwr.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rcc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rcc_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rtc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rtc_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_sd.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_smartcard.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_spi.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_spi_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_sram.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_tim.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_tim_ex.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_uart.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_usart.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_wwdg.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_crc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_exti.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_fsmc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_gpio.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_pwr.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_rcc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_sdmmc.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_usb.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_utils.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm_spi_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/system_stm32f1xx.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/trng_api.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/us_ticker_16b.o \
../Core/Inc/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/us_ticker_32b.o 


# Each subdirectory must supply rules for building sources it contributes

