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
#include "bldc.h"
#include <stdio.h>

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
extern P    rtP_Right;
// ###############################################################################

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;
volatile int buzzerState = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

// PWM
static const uint16_t pwm_res  = PWM_PER; // = 2000
volatile int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */
uint16_t midpoint = PWM_PER / 2;

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

// Calibration
motor_t motorL = {
                  .num = 0, 
                  .htim = &htim_left,
                  .hall_port = LEFT_HALL_U_PORT,
                  .hallA_pin = LEFT_HALL_U_PIN,
                  .hallB_pin = LEFT_HALL_V_PIN,
                  .hallC_pin = LEFT_HALL_W_PIN,
                  .ADC1_trigger = ADC1_2_EXTERNALTRIGINJEC_T8_CC4,
                  .ADC2_trigger = ADC1_2_EXTERNALTRIGINJEC_T8_CC4,
                  .ADC3_trigger = ADC3_EXTERNALTRIGINJEC_T8_CC4,
                  .i_phaAB_ch  = ADC_CHANNEL_LEFT_U<<ADC_JSQR_JSQ4_Pos,
                  .i_phaBC_ch  = ADC_CHANNEL_LEFT_V<<ADC_JSQR_JSQ4_Pos,
                  .i_DCLink_ch = ADC_CHANNEL_LEFT_DC<<ADC_JSQR_JSQ4_Pos,
                  .enable = MOTOR_LEFT_ENA,
                  //.rtM = &rtM_Left,
                  .rtY = &rtY_Left, 
                  .rtU = &rtU_Left, 
                  .rtP = &rtP_Left,
                  .samplingPoint = PWM_PER - 110,
                  .dt_comp = DT_COMP,
                  .dt_comp_thres = DT_COMP_THRES
                  };
motor_t motorR = {
                  .num = 1,
                  .htim = &htim_right,
                  .hall_port = RIGHT_HALL_U_PORT,
                  .hallA_pin = RIGHT_HALL_U_PIN,
                  .hallB_pin = RIGHT_HALL_V_PIN,
                  .hallC_pin = RIGHT_HALL_W_PIN,
                  .ADC1_trigger = ADC1_2_3_EXTERNALTRIGINJEC_T1_CC4,
                  .ADC2_trigger = ADC1_2_3_EXTERNALTRIGINJEC_T1_CC4,
                  .ADC3_trigger = ADC1_2_3_EXTERNALTRIGINJEC_T1_CC4,
                  .i_phaAB_ch  = ADC_CHANNEL_RIGHT_U<<ADC_JSQR_JSQ4_Pos,
                  .i_phaBC_ch  = ADC_CHANNEL_RIGHT_V<<ADC_JSQR_JSQ4_Pos,
                  .i_DCLink_ch = ADC_CHANNEL_RIGHT_DC<<ADC_JSQR_JSQ4_Pos,
                  .enable = MOTOR_RIGHT_ENA,
                  //.rtM = &rtM_Right,
                  .rtY = &rtY_Right,
                  .rtU = &rtU_Right,
                  .rtP = &rtP_Right,
                  .samplingPoint = PWM_PER - 110
                  };
volatile uint8_T motorN;
uint8_t phase_balancing = PHASE_BALANCING;

void TIM1_UP_IRQHandler(){
  // This runs once per TIM1 cycles because of the repetition counter

  WRITE_REG(TIM1->SR, ~(TIM_SR_UIF));
  // Run buzzer here as CC4 and ADC interrupts stop when timer output is disabled
  runBuzzer();

  // Start regular ADC
  SET_BIT(ADC1->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
}

void ADC1_2_IRQHandler(void){
  GPIOA->BSRR = GPIO_BSRR_BS2; // Output high
  __disable_irq();
  // Reconfigure ADC triggers and channels to sample the alternate motor
  switchADC();

  // Run FOC calculation and update PWM
  runFOC();

  __HAL_ADC_CLEAR_FLAG(&hadc1, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
  __enable_irq();
  GPIOA->BRR = GPIO_BRR_BR2; // Output low
}

void switchADC(void){
  // Disable External Trigger for Injected ADC 
  CLEAR_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);
  CLEAR_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG);
  CLEAR_BIT(ADC3->CR2, ADC_CR2_JEXTTRIG);

  // Setup the new External triggers for injected ADC
  motorN = (READ_BIT(ADC1->CR2, ADC_CR2_JEXTSEL) == ADC1_2_3_EXTERNALTRIGINJEC_T1_CC4);
  if (motorN == 1){
    if (motorL.enable){
      // Set ADC injected channels for Left Motor
      ADC1->JSQR = motorL.i_phaAB_ch;
      ADC2->JSQR = motorL.i_phaBC_ch;
      ADC3->JSQR = motorL.i_DCLink_ch;
      // Set ADC triggers for Left Motor (TIM8-CC4)
      MODIFY_REG(ADC1->CR2,ADC_CR2_JEXTSEL,motorL.ADC1_trigger);
      MODIFY_REG(ADC2->CR2,ADC_CR2_JEXTSEL,motorL.ADC2_trigger);
      MODIFY_REG(ADC3->CR2,ADC_CR2_JEXTSEL,motorL.ADC3_trigger);
    }
  }else{
    if (motorR.enable){
      // Set ADC injected channels for Right Motor
      ADC1->JSQR = motorR.i_phaAB_ch;
      ADC2->JSQR = motorR.i_phaBC_ch;
      ADC3->JSQR = motorR.i_DCLink_ch;
      // Set ADC triggers for Right Motor (TIM1-CC4)
      MODIFY_REG(ADC1->CR2,ADC_CR2_JEXTSEL,motorR.ADC1_trigger);
      MODIFY_REG(ADC2->CR2,ADC_CR2_JEXTSEL,motorR.ADC2_trigger);
      MODIFY_REG(ADC3->CR2,ADC_CR2_JEXTSEL,motorR.ADC3_trigger);
    }
  }

  // Enable External trigger for Injected ADC
  SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);
  SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG);
  SET_BIT(ADC3->CR2, ADC_CR2_JEXTTRIG);
}

void runFOC(void){
  
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  
  // ############################### MOTOR CONTROL ###############################

  static boolean_T OverrunFlag = false;
  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  enableFin = enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode;
  
  if (motorN == 0){
    if (motorL.enable){
      runMotor(&motorL);
    }else{
      LEFT_TIM->BDTR &= ~TIM_BDTR_MOE; // Turn PWM output off if motor disabled
    }
  }
  if (motorN == 1){
    if (motorR.enable){
      runMotor(&motorR);
    }else{
      RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; // Turn PWM output off if motor disabled
    }
  }
  
  /* Indicate task complete */
  OverrunFlag = false;
  
 // ###############################################################################
}

void runBuzzer(void){
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
      buzzerState = !buzzerState;
    }
  } else if (buzzerPrev) {
      buzzerState = buzzerPrev = 0;
  }
  BUZZER_PORT->BSRR = (uint32_t)(buzzerState ? BUZZER_PIN : BUZZER_PIN << 16U);
}

void runMotor(motor_t *motor){
  // ############################### MOTOR CONTROL ###############################

  // Works because all the halls from same motor are on the same GPIO port, so we can read whole port state in one go
  uint32_t tmp_IDR = (motor->hall_port)->IDR;

  /* Set motor inputs here */
  motor->rtU->b_motEna     = enableFin && motor->enable;
  motor->rtP->b_diagEna    = motor->test < 7 ? 0 : motor->enable; // Diag off during checks
  motor->rtU->z_ctrlModReq = ctrlModReq;
  motor->rtU->r_inpTgt     = motor->num == 0 ? pwml : pwmr;
  motor->rtU->b_hallA      = !(tmp_IDR & motor->hallA_pin);
  motor->rtU->b_hallB      = !(tmp_IDR & motor->hallB_pin);
  motor->rtU->b_hallC      = !(tmp_IDR & motor->hallC_pin);
  // rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
  
  // Test 0 is offset calibration, after that offset can be substracted from adc reading
  motor->rtU->i_phaAB      = (int16_t)(motor->test== 0 ? ADC1->JDR1 : motor->calib[0].i_phaAB  - ADC1->JDR1);
  motor->rtU->i_phaBC      = (int16_t)(motor->test== 0 ? ADC2->JDR1 : motor->calib[0].i_phaBC  - ADC2->JDR1);
  motor->rtU->i_DCLink     = (int16_t)(motor->test== 0 ? ADC3->JDR1 : motor->calib[0].i_DCLink - ADC3->JDR1);
  
  if (motor->test > 0){ // After offset calibration only
    // Disable PWM when current limit is reached (current chopping)
    // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
    if(ABS(motor->rtU->i_DCLink) > curDC_max) {
      motor->htim->Instance->BDTR &= ~TIM_BDTR_MOE;
    }
  }

  // Run calibration/tests
  calibration(motor);
  
  /* Step the controller */
  if (motor->num == 0){
    BLDC_controller_step(rtM_Left);
  }else{
    BLDC_controller_step(rtM_Right);
  }
  
  if (motor->test> 3){
    motor->duty[0] = motor->rtY->DC_phaA;
    motor->duty[1] = motor->rtY->DC_phaB;
    motor->duty[2] = motor->rtY->DC_phaC;
    for(int i=0;i<3;i++){
      // Compensation deadtime by adding or substracting the deadtime depending on the sign if value is higher than deadband 
      if (motor->rtU->z_ctrlModReq == VLT_MODE && motor->dt_comp > 0)  motor->duty[i] += ABS(motor->duty[i]) >= motor->dt_comp_thres ? SIGN(motor->duty[i]) * motor->dt_comp : MAP(motor->duty[i],0,motor->dt_comp_thres,0,motor->dt_comp);
      // Apply midpoint clamp and PWM Margin to allow current measurement for FOC
      motor->duty[i] = (uint16_t)CLAMP(motor->duty[i] + midpoint, pwm_margin, pwm_res-pwm_margin);
    }
  }

  // Set motor outputs
  motor->htim->Instance->CCR1  = motor->duty[0];
  motor->htim->Instance->CCR2  = motor->duty[1];
  motor->htim->Instance->CCR3  = motor->duty[2];
  motor->htim->Instance->CCR4  = motor->samplingPoint;

  // =================================================================
}

void enableMotors(uint8_t silent){
  if (motorR.enable){
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
    RIGHT_TIM->BDTR |= TIM_BDTR_AOE;
  }

  if (motorL.enable){
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    LEFT_TIM->BDTR |= TIM_BDTR_AOE;
  }
  
  if (silent) return;

  // make 2 beeps indicating the motor enable
  beepShort(6); beepShort(4);
  HAL_Delay(100);
  enable = 1; // enable motors
  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3) || defined(DEBUG_SEGGER_RTT)
  printf("-- Motors enabled --\r\n");
  #endif
}

void disableMotors(uint8_t silent){
  if (motorR.enable){
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_AOE;
  }
  
  if (motorL.enable){
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    LEFT_TIM->BDTR &= ~TIM_BDTR_AOE;
  }

  if (silent) return;

  // make 2 beeps indicating the motor disable
  beepShort(4); beepShort(6);
  HAL_Delay(100);
  enable = 0; // enable motors
  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3) || defined(DEBUG_SEGGER_RTT)
  printf("-- Motors disabled --\r\n");
  #endif
}

void calibration(motor_t *motor){
  #define SAMPLES   1000 // Number of samples for each test. Higher can overflow
  #define WAIT      1000 // Number of cycles to wait
  #define TEST_PWM  80   // PWM value for tests
  #define PHASE_CHECK_THRESHOLD 10 // Value must be higher than this
  #define END_ANGLE 3 * 16 * 360 
  uint8_t pwm = 0;
  uint8_t pos = (motor->rtU->b_hallA<<2) + (motor->rtU->b_hallB<<1) + motor->rtU->b_hallC;

  switch (motor->state){
    case 0:
      // TEST 0 - calibration phase current offset, all pwm's are 0
      motor->test  = 0;
      motor->state = 10; // Next State 
      break;
    case 1:
      // TEST 1 - power only phase U
      motor->test  = 1;
      motor->state = 10; // Next State
      break;
    case 2:
      // TEST 2 - power only phase V
      motor->test  = 2;
      motor->state = 10; // Next State
      break;
    case 3:
      // TEST 3 - power only phase W
      motor->test  = 3;
      motor->state = 10; // Next State
      break;
    case 4:
      // TEST 4 - hall autodetect
      motor->test  = 4;
      motor->rtU->b_motEna = motor->rtP->b_angleMeasEna = 1;
      motor->rtU->z_ctrlModReq = VLT_MODE;
      motor->rtU->r_inpTgt = TEST_PWM;
           
      switch (motor->substate){
        case 0:
          motor->autodetect_dir = 1;
          // Reinit HalltoAngle table
          for(int i = 0;i<8;i++) motor->vec_hallToAngle_Value[0][i] = motor->vec_hallToAngle_Value[1][i] = 0;

          motor->hallA_min = motor->hallB_min = motor->hallC_min = 1;
          motor->hallA_max = motor->hallB_max = motor->hallC_max = 0;

          motor->calib[motor->test].count = 0; // Reset counter for next substate
          motor->substate++;
          break;
        case 1:
          // Use angle input as a workaround to run in the motor in open mode, being able to target a particular electric angle
          motor->rtU->a_mechAngle = 0;
          motor->rtU->r_inpTgt = MAP(motor->calib[motor->test].count,0,WAIT,0,TEST_PWM);
          // Wait for motor to align
          if (motor->calib[motor->test].count++ == WAIT ) motor->substate++;
          break;
        case 2:
          if (( motor->autodetect_dir == 1 && (motor->rtU->a_mechAngle < END_ANGLE / motor->rtP->n_polePairs))||
              ( motor->autodetect_dir == -1 && (motor->rtU->a_mechAngle > 0))){
            
            // Wait for motor to align to this angle
            if (motor->calib[motor->test].count++ % 100 == 0){
              // Electrical angle without wraping around 360-0
              motor->a_elecAngle = (motor->rtU->a_mechAngle * motor->rtP->n_polePairs / 16);
              
              // Accumulate angle and increment count for hall position
              motor->vec_hallToAngle_Value[0][pos] += motor->a_elecAngle;
              motor->vec_hallToAngle_Value[1][pos]++;
            
              // Increment mechanical angle
              motor->rtU->a_mechAngle+= motor->autodetect_dir;
             
              // Save min/max state for hall
              motor->hallA_min = MIN(motor->hallA_min,motor->rtU->b_hallA);
              motor->hallB_min = MIN(motor->hallB_min,motor->rtU->b_hallB);
              motor->hallC_min = MIN(motor->hallC_min,motor->rtU->b_hallC);
              motor->hallA_max = MAX(motor->hallA_max,motor->rtU->b_hallA);
              motor->hallB_max = MAX(motor->hallB_max,motor->rtU->b_hallB);
              motor->hallC_max = MAX(motor->hallC_max,motor->rtU->b_hallC);            
            }
            
            //if ( motor->autodetect_dir == 1) motor->rtP->vec_hallToPos_Value[pos] = (motor->rtY->a_elecAngle+60)/60 % 6;
          }else{
            if (motor->autodetect_dir == 1){
              // Change direction
              motor->autodetect_dir = -1;
            }else{
              // End
              motor->substate++;
            }
          }
          break;
        case 3:
          // Calculate Average Angle for each hall position
          for(int i = 0;i<8;i++){
            motor->vec_hallToAngle_Value[0][i] = (motor->vec_hallToAngle_Value[0][i] / motor->vec_hallToAngle_Value[1][i]) % 360;
            if (motor->vec_hallToAngle_Value[1][i] != 0){
              motor->rtP->vec_hallToPos_Value[i] = ( ( motor->vec_hallToAngle_Value[0][i] + 60) / 60 ) % 6; 
            }
          }
        
          // Check if hall state changed
          motor->hallA_ok = motor->hallA_min != motor->hallA_max; 
          motor->hallB_ok = motor->hallB_min != motor->hallB_max;
          motor->hallC_ok = motor->hallC_min != motor->hallC_max;

          motor->calib[motor->test].count = 0;
          motor->substate++;
          break;
        case 4:
          motor->rtU->r_inpTgt = MAP(motor->calib[motor->test].count,0,WAIT,TEST_PWM,0);
          
          if (motor->calib[motor->test].count++ == WAIT ) {
            motor->rtP->b_angleMeasEna = 0;
            motor->substate = 0;
            motor->state++;
          }
          break;
      }
      break;
    case 5:
      // Print results
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3) || defined(DEBUG_SEGGER_RTT)
        printf("\r\n %s Motor\r\n",motor->num==0 ? "Left " : "Right ");
        printf("+-------+-----+------+------+-----+-----+\r\n");
        printf("|-------|DCCUR|PHA-AB|PHA-BC|DCCUR|PHASE|\r\n");
        printf("+-------+-----+------+------+-----+-----+\r\n");
        for(int i=0;i<(motor->test);i++){
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
          
          printf("|%05li|%06li|%06li|%s|%s|\r\n",
                 motor->calib[i].i_DCLink,
                 motor->calib[i].i_phaAB,
                 motor->calib[i].i_phaBC,
                 motor->calib[i].dccheck    == -1 ? "_____" : motor->calib[i].dccheck    == 1 ? "___OK" : "__NOK",
                 motor->calib[i].phasecheck == -1 ? "_____" : motor->calib[i].phasecheck == 1 ? "___OK" : "__NOK"
                );
        }
        if (motor->phaseSwapped) printf("Phase current swapped\r\n");
        printf("\r\n");
        
        printf("Hall Positions\r\n");
        for(int i = 0;i<8;i++){
          printf("%i,",motor->rtP->vec_hallToPos_Value[i]);
        }
        printf("\r\nHall Angles\r\n");
        for(int i = 0;i<8;i++){
          printf("%i,",motor->vec_hallToAngle_Value[0][i]);
        }  
        printf("\r\n\r\n");

        printf("Hall A  B  C \r\n");
        printf("Min- %i %i %i\r\n",motor->hallA_min,motor->hallB_min,motor->hallC_min);
        printf("Max- %i %i %i\r\n",motor->hallA_max,motor->hallB_max,motor->hallC_max);
        printf("OK?- %i %i %i\r\n",motor->hallA_ok ,motor->hallB_ok ,motor->hallC_ok);
        
      #endif
      motor->state++; // Next State
      break;
    case 6:
      // Process errors
      if (motor->hallA_max + motor->hallB_max + motor->hallC_max == 0){
        // all halls are always low, cable not plugged ? missing 5v or GND ?
        motor->rtY->z_errCode = 3;
      }else if ( motor->hallA_max + motor->hallB_max + motor->hallC_max < 3 ){
        // at least one hall is always low, it's defective
        motor->rtY->z_errCode = 1;
      }else if ( motor->hallA_min + motor->hallB_min + motor->hallC_min > 0 ){
        // at least one hall is always high, it's shorted
        motor->rtY->z_errCode = 2;
      }

      motor->state++; // Next State
      break;
    case 7:
      // End
      break;
    case 10:
      // Initialize counter
      motor->calib[motor->test].count = 0;
      motor->state++; // Next State
      break;
    case 11:
      // Increase pwm until reaching TEST_PWM for each Phase depending on test number
      pwm = MAP(motor->calib[motor->test].count,0, WAIT, 0, TEST_PWM);
      motor->duty[0] = (motor->test == 1) ? pwm : 0;
      motor->duty[1] = (motor->test == 2) ? pwm : 0;
      motor->duty[2] = (motor->test == 3) ? pwm : 0;
      if (motor->calib[motor->test].count++ == WAIT - 1) motor->state++;
      break;  
    case 12:
      // Initialize variables
      motor->calib[motor->test].count      = 0;
      motor->calib[motor->test].i_phaAB    = 0;
      motor->calib[motor->test].i_phaBC    = 0;
      motor->calib[motor->test].i_DCLink   = 0;
      motor->calib[motor->test].dccheck    = -1;
      motor->calib[motor->test].phasecheck = -1;
      motor->state++;
      break;
    case 13:
      // Integrate ADC values
      motor->calib[motor->test].i_phaAB  += motor->rtU->i_phaAB;
      motor->calib[motor->test].i_phaBC  += motor->rtU->i_phaBC;
      motor->calib[motor->test].i_DCLink += motor->rtU->i_DCLink;
      if (motor->calib[motor->test].count++ == SAMPLES-1) motor->state++;
      break; 
    case 14:
      // Calculate average
      motor->calib[motor->test].i_phaAB  /= SAMPLES;
      motor->calib[motor->test].i_phaBC  /= SAMPLES;
      motor->calib[motor->test].i_DCLink /= SAMPLES;
      
      if (IN_RANGE(motor->test,1,3)){
        motor->calib[motor->test].dccheck = (motor->calib[motor->test].i_DCLink < 0);
        
        if (motor->num == 0 && motor->test == 1 ) motor->calib[motor->test].phasecheck = (motor->calib[motor->test].i_phaAB > PHASE_CHECK_THRESHOLD);
        if (motor->num == 0 && motor->test == 2 ) motor->calib[motor->test].phasecheck = (motor->calib[motor->test].i_phaBC > PHASE_CHECK_THRESHOLD);
        if (motor->num == 1 && motor->test == 2 ) motor->calib[motor->test].phasecheck = (motor->calib[motor->test].i_phaAB > PHASE_CHECK_THRESHOLD);
        if (motor->num == 1 && motor->test == 3 ) motor->calib[motor->test].phasecheck = (motor->calib[motor->test].i_phaBC > PHASE_CHECK_THRESHOLD);
      }

      if (motor->test == 3 &&
         (( motor->num == 0 && 
           motor->calib[1].phasecheck == 0 && 
           motor->calib[2].phasecheck == 0 && 
           motor->calib[1].i_phaBC > PHASE_CHECK_THRESHOLD &&
           motor->calib[2].i_phaAB > PHASE_CHECK_THRESHOLD) || 
         ( motor->num == 1 && 
           motor->calib[2].phasecheck == 0 && 
           motor->calib[3].phasecheck == 0 && 
           motor->calib[2].i_phaBC > PHASE_CHECK_THRESHOLD &&
           motor->calib[3].i_phaAB > PHASE_CHECK_THRESHOLD))) {
        // Phase current are swapped
        uint32_t tmp_ch,i_phaTMP;
        // Swap ADC channels
        tmp_ch = motor->i_phaAB_ch;
        motor->i_phaAB_ch = motor->i_phaBC_ch;
        motor->i_phaBC_ch = tmp_ch;
        // Swap Offsets
        i_phaTMP = motor->calib[0].i_phaAB;
        motor->calib[0].i_phaAB = motor->calib[0].i_phaBC;
        motor->calib[0].i_phaBC = i_phaTMP;

        motor->phaseSwapped = 1;
        motor->test = 1; // Redo the test to clear the checks  
      }else{
        if (motor->test == 0){
          // Only Offset calibration at startup
          motor->test++; // = 7;
        }else{
          // Autodetect
          motor->test++;
        }
      }
      motor->state = motor->test;

      break;
    default:
       break;
  }     
}

