#pragma once
#include <hal/nrf_gpio.h>
#include <nrfx_saadc.h>

// BSP:

#define PIN_POWER_ENABLE       NRF_GPIO_PIN_MAP(1, 14)
#define PIN_INT_BUTTON_HOME    NRF_GPIO_PIN_MAP(1, 13)

// Communication:
// -- SPI: SDCARD MEM, STIM DAC, EEG ADC
// -- I2C: PPG, ALS
// -- Interrupts: SDCARD, PPG, ALS, Side Connector
// -- Control: EEG/Stim Switch, EEG Enable, Stim Enable

#define PIN_SPI_MISO           NRF_GPIO_PIN_MAP(2, 9)
#define PIN_SPI_MOSI           NRF_GPIO_PIN_MAP(2, 8)
#define PIN_SPI_CLK            NRF_GPIO_PIN_MAP(2, 6)
#define PIN_SPI_CS_SDCARD      NRF_GPIO_PIN_MAP(2, 3)
#define PIN_SPI_CS_STIM_DAC    NRF_GPIO_PIN_MAP(2, 5)
#define PIN_SPI_CS_EEG         NRF_GPIO_PIN_MAP(2, 0)

#define PIN_I2C_SDA            NRF_GPIO_PIN_MAP(2, 4)
#define PIN_I2C_CLK            NRF_GPIO_PIN_MAP(2, 2)

#define PIN_INT_SDCARD         NRF_GPIO_PIN_MAP(1, 8)
#define PIN_INT_PPG            NRF_GPIO_PIN_MAP(2, 7)
#define PIN_INT_ALS            NRF_GPIO_PIN_MAP(2, 10)
#define PIN_INT_SIDE_CONN      NRF_GPIO_PIN_MAP(1, 4)

#define PIN_EEGSTIM_SWITCH     NRF_GPIO_PIN_MAP(1, 5)
#define PIN_EEG_ENABLE         NRF_GPIO_PIN_MAP(2, 1)
#define PIN_STIM_ENABLE        NRF_GPIO_PIN_MAP(1, 11)

// Communication:
// -- Audio: 高通 BLE TWS Module.

#define PIN_AUDIO_TX           NRF_GPIO_PIN_MAP(0, 1)
#define PIN_AUDIO_RX           NRF_GPIO_PIN_MAP(0, 0)
#define PIN_INT_AUDIO_STATUS_CONN   NRF_GPIO_PIN_MAP(0, 2)
#define PIN_INT_AUDIO_STATUS_BLE    NRF_GPIO_PIN_MAP(0, 3)
#define PIN_INT_AUDIO_STATUS_PWR    NRF_GPIO_PIN_MAP(0, 4)

#define PIN_AUDIO_PWR_ENABLE   NRF_GPIO_PIN_MAP(1, 9)
#define PIN_AUDIO_AMP_ENABLE   NRF_GPIO_PIN_MAP(1, 10)

// Sensors: ADC.

#define PIN_ADC_EEG1           NRFX_ANALOG_EXTERNAL_AIN2 // P1.06
#define PIN_ADC_EEG2           NRFX_ANALOG_EXTERNAL_AIN3 // P1.07
#define PIN_ADC_BATTERY        NRFX_ANALOG_EXTERNAL_AIN5 // P1.12
