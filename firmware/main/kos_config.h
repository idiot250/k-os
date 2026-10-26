#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/*
 * K-OS hardware configuration.
 *
 * Confirmed from the prior conversation:
 *   PCM5102A: BCLK 41, LRCK/WS 40, DIN 39
 *   KY-040:   CLK 6, DT 7, SW 15
 *   Buttons:  16, 17, 18, 8
 *
 * IMPORTANT: the exact potentiometer wiper pins were not retained in the
 * available context. GPIO4 and GPIO5 are the best reconstruction of the
 * earlier schematic guidance.
 * If your schematic actually used other pins, change both the GPIO and channel
 * values below to match it. GPIO4 = ADC1_CH3; GPIO5 = ADC1_CH4.
 */
#define KOS_I2S_BCLK_GPIO       GPIO_NUM_41
#define KOS_I2S_LRCK_GPIO       GPIO_NUM_40
#define KOS_I2S_DOUT_GPIO       GPIO_NUM_39

#define KOS_ENCODER_CLK_GPIO    GPIO_NUM_6
#define KOS_ENCODER_DT_GPIO     GPIO_NUM_7
#define KOS_ENCODER_SW_GPIO     GPIO_NUM_15

#define KOS_BUTTON_MODE_GPIO    GPIO_NUM_16
#define KOS_BUTTON_ENGINE_GPIO  GPIO_NUM_17
#define KOS_BUTTON_VARIATION_GPIO GPIO_NUM_18
#define KOS_BUTTON_FX_GPIO      GPIO_NUM_8

#define KOS_POT_CUTOFF_GPIO     GPIO_NUM_4
#define KOS_POT_CUTOFF_CHANNEL  ADC_CHANNEL_3
#define KOS_POT_INTENSITY_GPIO  GPIO_NUM_5
#define KOS_POT_INTENSITY_CHANNEL ADC_CHANNEL_4

/* The firmware assumes buttons/encoder switches close to GND when pressed. */
#define KOS_SWITCHES_ACTIVE_LOW 1
