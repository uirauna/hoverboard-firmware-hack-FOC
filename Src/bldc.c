/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
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

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include <stdio.h>

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
// ###############################################################################

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000

static int32_T offset[2][4][7];
static uint8_T calibState[2];
static uint8_T test[2];
static uint16_t offsetcount[2];
void calibration(int8_t motor);
void applyPWM(uint8_t motor,uint16_t u,uint16_t v,uint16_t w,uint8_t midclamp);
uint16_t ul,vl,wl,ur,vr,wr;


int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) {

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  calibration(0);
  calibration(1);
  if (calibState[0] != 5 && calibState[1] != 5){
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_DC   = (int16_t)(offset[0][0][0] - adc_buffer.dcl);
  curL_phaA = (int16_t)(offset[0][0][1] - adc_buffer.rlA) / 6 * 5;
  curL_phaB = (int16_t)(offset[0][0][2] - adc_buffer.rlB);
  
  // Get Right motor currents
  curR_DC   = (int16_t)(offset[1][0][0] - adc_buffer.dcr);
  curR_phaB = (int16_t)(offset[1][0][2] - adc_buffer.rrB);
  curR_phaC = (int16_t)(offset[1][0][3] - adc_buffer.rrC);
  

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

   
  if(ABS(curR_DC)  > curDC_max || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    #ifdef MOTOR_RIGHT_ENA
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
    #endif
  }
  

  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else if (buzzerPrev) {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
      buzzerPrev = 0;
  }

  // Adjust pwm_margin depending on the selected Control Type
  if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################
  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  enableFin = enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode;
 
  #ifdef MOTOR_LEFT_ENA
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    /* Set motor inputs here */
    rtU_Left.b_motEna     = enableFin;
    rtU_Left.z_ctrlModReq = ctrlModReq;  
    rtU_Left.r_inpTgt     = pwml;
    rtU_Left.b_hallA      = hall_ul;
    rtU_Left.b_hallB      = hall_vl;
    rtU_Left.b_hallC      = hall_wl;
    rtU_Left.i_phaAB      = curL_phaA;
    rtU_Left.i_phaBC      = curL_phaB;
    rtU_Left.i_DCLink     = curL_DC;
    // rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    
    /* Step the controller */
    BLDC_controller_step(rtM_Left);

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
    // errCodeLeft  = rtY_Left.z_errCode;
    // motSpeedLeft = rtY_Left.n_mot;
    // motAngleLeft = rtY_Left.a_elecAngle;

    applyPWM(0,ul,vl,wl,1);
    // LEFT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // LEFT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // LEFT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);

  // =================================================================
  #endif

  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

  #ifdef MOTOR_RIGHT_ENA
  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    /* Set motor inputs here */
    rtU_Right.b_motEna      = enableFin;
    rtU_Right.z_ctrlModReq  = ctrlModReq;
    rtU_Right.r_inpTgt      = pwmr;
    rtU_Right.b_hallA       = hall_ur;
    rtU_Right.b_hallB       = hall_vr;
    rtU_Right.b_hallC       = hall_wr;
    rtU_Right.i_phaAB       = curR_phaB;
    rtU_Right.i_phaBC       = curR_phaC;
    rtU_Right.i_DCLink      = curR_DC;
    // rtU_Right.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    
    /* Step the controller */
    BLDC_controller_step(rtM_Right);

    /* Get motor outputs here */
    ur            = rtY_Right.DC_phaA;
    vr            = rtY_Right.DC_phaB;
    wr            = rtY_Right.DC_phaC;
    // errCodeRight  = rtY_Right.z_errCode;
    // motSpeedRight = rtY_Right.n_mot;
    // motAngleRight = rtY_Right.a_elecAngle;

    /* Apply commands */
    applyPWM(1,ur,vr,wl,1);
    // RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  
  // =================================================================
  #endif

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}

void calibration(int8_T motor){
  #define SAMPLES 1000   // Number of samples for each test
  #define WAIT_HALL 1000 // Number of cycles to wait
  #define PWM   150      // PWM value for tests
  
  #define DC_CHECK    // Each phase will be energized and DC current will be measured
  #define PHASE_CHECK // Each phase will be energized and phase current will be measured
  #define HALL_CHECK  // Each phase will be energized for a longer time and hall sensor will be measured
  
  switch (calibState[motor]){
    case 0:
      // TEST 0
      // Turn PWM ON for all phased for current offset measurement
      applyPWM(motor,0,0,0,1);
      calibState[motor]=10;
      test[motor] = 0;
      break;
    case 1:
      // TEST 1
      // Set PWM for phase U only
      applyPWM(motor,PWM,0,0,0);
      calibState[motor]=10;
      test[motor] = 1;
      break;
    case 2:
      //TEST 2
      // Set PWM for phase V only
      applyPWM(motor,0,PWM,0,0);
      calibState[motor]=10;
      test[motor] = 2;
      break;
    case 3:
      //TEST 3
      // Set PWM for phase W only
       applyPWM(motor,0,0,PWM,0);
      calibState[motor]=10;
      test[motor] = 3;
      break;
    case 4:
      // Print results
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("\r\n");
        printf(motor==0 ? "Left " : "Right ");
        printf("Motor\r\n");
        printf("+-------+-----+-----+-----+-----+-----+-----+-----+-----+-----+----+\r\n");
        printf("|-------|DCCUR|PHA-A|PHA-B|PHA-C|HALLA|HALLB|HALLC|DCCUR|PHASE|HALL|\r\n");
        printf("+-------+-----+-----+-----+-----+-----+-----+-----+-----+-----+----+\r\n");

        int i,j;
        #if defined(PHASE_CHECK) || defined(HALL_CHECK)
        for(i=0;i<4;i++){
        #else
        for(i=0;i<1;i++){
        #endif
          switch (i){
            case 0:
              printf("|Offsets");
              break;
            case 1:
              printf("|Phase A");
              break;
            case 2:
              printf("|Phase B");
              break;
            case 3:
              printf("|Phase C");
              break;
          } 
          for(j=0;j<7;j++){
            printf("|%05i",offset[motor][i][j]);
          }
          // Check phase current value
          if (i>0){ // First test is for offsets
            #if defined(DC_CHECK)
            printf(offset[motor][i][0] < 0 ? "|___OK" : "|__NOK"); // Is DC current high enough
            #else
            printf("|_____");
            #endif

            #if defined(PHASE_CHECK)
            // Check only valid phases for each motor
            if (!(motor == 0 && i ==3)  && !(motor == 1 && i ==1)){
              printf(offset[motor][i][i] > 5 ? "|___OK" : "|__NOK"); // Is Phase current high enough
            }else{
              printf("|_____");
            }
            #else
              printf("|_____");
            #endif

            #if defined(HALL_CHECK)
            uint8_t hallpos = (uint8_t)((offset[motor][i][4]<<2) + (offset[motor][i][5]<<1) + offset[motor][i][6]);
            uint8_t hallmap[4] = {0,6,3,5}; // Expected positions
            printf(hallpos == hallmap[i] ? "|__OK" : "|_NOK"); // Is position the expected one
            #else
            printf("|____"); // Is position the expected one
            #endif
          }else{
            printf("|_____|_____|____");
          }
          printf("|\r\n+-------+-----+-----+-----+-----+-----+-----+-----+-----+-----+----+\r\n");
        }
        printf("\r\n");
      #endif
      calibState[motor]++;
      break;
    case 5:
      // No more tests
      return;
      break;
    case 10:
      // Initialize counter
      offsetcount[motor] = 0;
      #ifdef HALL_CHECK
      calibState[motor]++;
      #else
      calibState[motor]+=2; // Skip next state if hal check is not needed
      #endif
      break;
    case 11:
      // Give enough time for the wheel to spin, only necessary for HALL_CHECK
      if (offsetcount[motor] == WAIT_HALL) calibState[motor]++;
      offsetcount[motor]++;
      break;  
    case 12:
      // Initialize variables
      offsetcount[motor] = 0;
      offset[motor][test[motor]][0] = 0;
      offset[motor][test[motor]][1] = 0;
      offset[motor][test[motor]][2] = 0;
      offset[motor][test[motor]][3] = 0;

      #if defined(HALL_CHECK)
      if (test[motor]>0){
        // Record all sensor state
        offset[motor][test[motor]][4] = motor==0 ? !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN) : !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
        offset[motor][test[motor]][5] = motor==0 ? !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN) : !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
        offset[motor][test[motor]][6] = motor==0 ? !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN) : !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);
      }
      #endif

      calibState[motor]++;
      break;
    case 13:
      // Sum of ADC values
      offset[motor][test[motor]][0] += motor == 0 ? adc_buffer.dcl : adc_buffer.dcr;
      offset[motor][test[motor]][1] += motor == 0 ? adc_buffer.rlA : 0;
      offset[motor][test[motor]][2] += motor == 0 ? adc_buffer.rlB : adc_buffer.rrB;
      offset[motor][test[motor]][3] += motor == 0 ? 0              : adc_buffer.rrC;
      offsetcount[motor] ++;
      if (offsetcount[motor] == SAMPLES) calibState[motor]++;
      break; 
    case 14:
      // Calculate average
      offset[motor][test[motor]][0] /= SAMPLES;
      offset[motor][test[motor]][1] /= SAMPLES;
      offset[motor][test[motor]][2] /= SAMPLES;
      offset[motor][test[motor]][3] /= SAMPLES;
      if (test[motor]>0){
         // Susbtract from offset if it's not the first test
         offset[motor][test[motor]][0] = (int32_t) (offset[motor][0][0] - offset[motor][test[motor]][0]);
         offset[motor][test[motor]][1] = (int32_t) (offset[motor][0][1] - offset[motor][test[motor]][1]);
         offset[motor][test[motor]][2] = (int32_t) (offset[motor][0][2] - offset[motor][test[motor]][2]);
         offset[motor][test[motor]][3] = (int32_t) (offset[motor][0][3] - offset[motor][test[motor]][3]);
      }

      test[motor]++;
      #if defined(DC_CHECK) || defined(PHASE_CHECK) || defined(HALL_CHECK)
        calibState[motor] = test[motor];
      #else
        calibState[motor] = 4; // If Phase check is not needed, skip test 1-2-3
      #endif
      break;
    default:
       break;
  }
  
}


void applyPWM(uint8_t motor,uint16_t u,uint16_t v,uint16_t w,uint8_t midclamp){
  
  #define DEADTIME_COMPENSATION 0
  #define DEADTIME_DEADBAND 10

  // Compensation deadtime by adding or substracting the deadtime depending on the sign if value is higher than deadband 
  if (DEADTIME_COMPENSATION > 0){
    u += (uint16_t)(ABS(u) > DEADTIME_DEADBAND ? SIGN(u) * DEADTIME_COMPENSATION : 0);
    v += (uint16_t)(ABS(v) > DEADTIME_DEADBAND ? SIGN(v) * DEADTIME_COMPENSATION : 0);
    w += (uint16_t)(ABS(w) > DEADTIME_DEADBAND ? SIGN(w) * DEADTIME_COMPENSATION : 0);
  }

  // Apply midpoint clamp
  if (midclamp){
    u = (uint16_t)CLAMP(u + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    v = (uint16_t)CLAMP(v + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    w = (uint16_t)CLAMP(w + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  }

  if (motor==0){
     // motor 0 is left one
     LEFT_TIM->LEFT_TIM_U  = u;
     LEFT_TIM->LEFT_TIM_V  = v;
     LEFT_TIM->LEFT_TIM_W  = w;
  }else if(motor==1){
     // motor 1 is right one
     RIGHT_TIM->RIGHT_TIM_U  = u;
     RIGHT_TIM->RIGHT_TIM_V  = v;
     RIGHT_TIM->RIGHT_TIM_W  = w;
  }
}

