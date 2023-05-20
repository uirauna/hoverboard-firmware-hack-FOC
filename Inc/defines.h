/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"


// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define MCU_FREQ         64000000     // MCU frequency
#define PWM_FREQ            16000     // PWM frequency in Hz / is also used for buzzer
#define PWM_PER MCU_FREQ/2/PWM_FREQ   // PWM Period
#define DEAD_TIME              48     // PWM deadtime
#ifdef VARIANT_TRANSPOTTER
  #define DELAY_IN_MAIN_LOOP    2
#else
  #define DELAY_IN_MAIN_LOOP    5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#endif
#define TIMEOUT                20     // number of wrong / missing input commands before emergency off
#define DC_SHUNT               0.0035 // Resistance of DC shunt in ohms, 0.0035 for 2 R007 shunts 
#define OPAMP_FACTOR           11 
#define A2BIT_CONV             DC_SHUNT * OPAMP_FACTOR * 4095 / 3.3 // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc
// #define PRINTF_FLOAT_SUPPORT          // [-] Uncomment this for printf to support float on Serial Debug. It will increase code size! Better to avoid it!

// ADC conversion time definitions
#define ADC_CONV_TIME_1C5       (14)  //Total ADC clock cycles / conversion = (  1.5+12.5)
#define ADC_CONV_TIME_7C5       (20)  //Total ADC clock cycles / conversion = (  7.5+12.5)
#define ADC_CONV_TIME_13C5      (26)  //Total ADC clock cycles / conversion = ( 13.5+12.5)
#define ADC_CONV_TIME_28C5      (41)  //Total ADC clock cycles / conversion = ( 28.5+12.5)
#define ADC_CONV_TIME_41C5      (54)  //Total ADC clock cycles / conversion = ( 41.5+12.5)
#define ADC_CONV_TIME_55C5      (68)  //Total ADC clock cycles / conversion = ( 55.5+12.5)
#define ADC_CONV_TIME_71C5      (84)  //Total ADC clock cycles / conversion = ( 71.5+12.5)
#define ADC_CONV_TIME_239C5     (252) //Total ADC clock cycles / conversion = (239.5+12.5)

// This settings influences the actual sample-time. Only use definitions above
// This parameter needs to be the same as the ADC conversion for Current Phase of the FIRST Motor in setup.c
#define ADC_CONV_CLOCK_CYCLES   (ADC_CONV_TIME_1C5)

// Set the configured ADC divider. This parameter needs to be the same ADC divider as PeriphClkInit.AdcClockSelection (see main.c)
#define ADC_CLOCK_DIV           (4)

// ADC Total conversion time: this will be used to offset TIM8 in advance of TIM1 to align the Phase current ADC measurement
// This parameter is used in setup.c
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES) // = ((SystemCoreClock / ADC_CLOCK_HZ) * ADC_CONV_CLOCK_CYCLES), where ADC_CLOCK_HZ = SystemCoreClock/ADC_CLOCK_DIV
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################


#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_U_PORT GPIOB
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_V_PORT GPIOB
#define LEFT_HALL_W_PIN GPIO_PIN_7
#define LEFT_HALL_W_PORT GPIOB

#define RIGHT_HALL_U_PIN GPIO_PIN_10
#define RIGHT_HALL_U_PORT GPIOC
#define RIGHT_HALL_V_PIN GPIO_PIN_11
#define RIGHT_HALL_V_PORT GPIOC
#define RIGHT_HALL_W_PIN GPIO_PIN_12
#define RIGHT_HALL_W_PORT GPIOC

#define LEFT_TIM TIM8
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB

#define RIGHT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_8
#define RIGHT_TIM_UH_PORT GPIOA
#define RIGHT_TIM_UL_PIN GPIO_PIN_13
#define RIGHT_TIM_UL_PORT GPIOB
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_9
#define RIGHT_TIM_VH_PORT GPIOA
#define RIGHT_TIM_VL_PIN GPIO_PIN_14
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_10
#define RIGHT_TIM_WH_PORT GPIOA
#define RIGHT_TIM_WL_PIN GPIO_PIN_15
#define RIGHT_TIM_WL_PORT GPIOB


#define LEFT_DC_CUR_PIN GPIO_PIN_0
#define LEFT_DC_CUR_PORT GPIOC
#define ADC_CHANNEL_LEFT_DC ADC_CHANNEL_10

#define LEFT_U_CUR_PIN GPIO_PIN_0
#define LEFT_U_CUR_PORT GPIOA
#define ADC_CHANNEL_LEFT_U ADC_CHANNEL_0

#define LEFT_V_CUR_PIN GPIO_PIN_3
#define LEFT_V_CUR_PORT GPIOC
#define ADC_CHANNEL_LEFT_V ADC_CHANNEL_13

#define RIGHT_DC_CUR_PIN GPIO_PIN_1
#define RIGHT_DC_CUR_PORT GPIOC
#define ADC_CHANNEL_RIGHT_DC ADC_CHANNEL_11

#define RIGHT_U_CUR_PIN GPIO_PIN_4
#define RIGHT_U_CUR_PORT GPIOC
#define ADC_CHANNEL_RIGHT_U ADC_CHANNEL_14

#define RIGHT_V_CUR_PIN GPIO_PIN_5
#define RIGHT_V_CUR_PORT GPIOC
#define ADC_CHANNEL_RIGHT_V ADC_CHANNEL_15

#if BOARD_VARIANT == 1
  #define VBAT GPIO_PIN_1
  #define VBAT_PORT GPIOA
  #define ADC_CHANNEL_VBAT ADC_CHANNEL_1
#else
  #define VBAT_PIN GPIO_PIN_2
  #define VBAT_PORT GPIOC
  #define ADC_CHANNEL_VBAT ADC_CHANNEL_12
#endif

// #define DCLINK_PULLUP 30000
// #define DCLINK_PULLDOWN 1000

#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB

#if BOARD_VARIANT == 1
  #define BUZZER_PIN GPIO_PIN_13
  #define BUZZER_PORT GPIOC
#else
  #define BUZZER_PIN GPIO_PIN_4
  #define BUZZER_PORT GPIOA
#endif

#if BOARD_VARIANT == 1
  #define OFF_PIN GPIO_PIN_15
  #define OFF_PORT GPIOC
#else
  #define OFF_PIN GPIO_PIN_5
  #define OFF_PORT GPIOA
#endif

#if BOARD_VARIANT == 1
  #define BUTTON_PIN GPIO_PIN_9
  #define BUTTON_PORT GPIOB
#else
  #define BUTTON_PIN GPIO_PIN_1
  #define BUTTON_PORT GPIOA
#endif
#define ADC_CHANNEL_BUTTON ADC_CHANNEL_1

#if BOARD_VARIANT == 1
  #define CHARGER_PIN GPIO_PIN_11
  #define CHARGER_PORT GPIOA
#else
  #define CHARGER_PIN GPIO_PIN_12
  #define CHARGER_PORT GPIOA
#endif

#if defined(CONTROL_PPM_LEFT)
  #define PPM_PIN             GPIO_PIN_3
  #define PPM_PORT            GPIOA
#elif defined(CONTROL_PPM_RIGHT)
  #define PPM_PIN             GPIO_PIN_11
  #define PPM_PORT            GPIOB
#endif

#if defined(CONTROL_PWM_LEFT)
  #define PWM_PIN_CH1         GPIO_PIN_2
  #define PWM_PORT_CH1        GPIOA
  #define PWM_PIN_CH2         GPIO_PIN_3
  #define PWM_PORT_CH2        GPIOA
#elif defined(CONTROL_PWM_RIGHT)
  #define PWM_PIN_CH1         GPIO_PIN_10
  #define PWM_PORT_CH1        GPIOB
  #define PWM_PIN_CH2         GPIO_PIN_11
  #define PWM_PORT_CH2        GPIOB
#endif

#if defined(SUPPORT_BUTTONS_LEFT)
  #define BUTTON1_PIN         GPIO_PIN_2
  #define BUTTON1_PORT        GPIOA
  #define BUTTON2_PIN         GPIO_PIN_3
  #define BUTTON2_PORT        GPIOA
#elif defined(SUPPORT_BUTTONS_RIGHT)
  #define BUTTON1_PIN         GPIO_PIN_10
  #define BUTTON1_PORT        GPIOB
  #define BUTTON2_PIN         GPIO_PIN_11
  #define BUTTON2_PORT        GPIOB
#endif

#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

#if defined(PRINTF_FLOAT_SUPPORT) && (defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)) && defined(__GNUC__)
    asm(".global _printf_float");     // this is the magic trick for printf to support float. Warning: It will increase code considerably! Better to avoid!
#endif


typedef struct {
  uint16_t batt1;
  uint16_t onoff;
  uint16_t l_tx2;
  uint16_t l_rx2;
  uint16_t temp;
  uint16_t vrefint;
} adc_buf_t;

typedef enum {
  NUNCHUK_CONNECTING,
  NUNCHUK_DISCONNECTED,
  NUNCHUK_RECONNECTING,
  NUNCHUK_CONNECTED
} nunchuk_state;

// Define I2C, Nunchuk, PPM, PWM functions
void I2C_Init(void);
nunchuk_state Nunchuk_Read(void);
void PPM_Init(void);
void PPM_ISR_Callback(void);
void PWM_Init(void);
void PWM_ISR_CH1_Callback(void);
void PWM_ISR_CH2_Callback(void);

// Sideboard definitions
#define LED1_SET            (0x01)
#define LED2_SET            (0x02)
#define LED3_SET            (0x04)
#define LED4_SET            (0x08)
#define LED5_SET            (0x10)
#define SENSOR1_SET         (0x01)
#define SENSOR2_SET         (0x02)
#define SENSOR_MPU          (0x04)

// RC iBUS switch definitions. Flysky FS-i6S has [SWA, SWB, SWC, SWD] = [2, 3, 3, 2] positions switch
#define SWA_SET             (0x0100)   //  0000 0001 0000 0000
#define SWB_SET             (0x0600)   //  0000 0110 0000 0000
#define SWC_SET             (0x1800)   //  0001 1000 0000 0000
#define SWD_SET             (0x2000)   //  0010 0000 0000 0000

#endif // DEFINES_H

