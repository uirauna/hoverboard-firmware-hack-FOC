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
#ifndef BLDC_H
#define BLDC_H

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

typedef struct
{
  uint16_t count;
  int32_t  i_DCLink;
  int32_t  i_phaAB;
  int32_t  i_phaBC;
  int8_t   dccheck;
  int8_t   phasecheck;
}calib_t;

typedef struct
{
  uint8_t num;
  // Autodetect
  calib_t calib[5];
  uint8_t test;
  uint8_t state;
  uint8_t substate;
  int8_t autodetect_dir;
  int16_t a_elecAngle;
  int32_T vec_hallToAngle_Value[2][8];
  uint8_t hallA_min;
  uint8_t hallB_min;
  uint8_t hallC_min;
  uint8_t hallA_max;
  uint8_t hallB_max;
  uint8_t hallC_max;
  uint8_t hallA_ok;
  uint8_t hallB_ok;
  uint8_t hallC_ok;
  uint8_t enable;
  // Timer/PWM
  TIM_HandleTypeDef *htim;
  int16_t duty[3];
  uint8_t dt_comp;
  uint8_t dt_comp_thres;
  // Hall
  GPIO_TypeDef *hall_port;
  uint16_t hallA_pin;
  uint16_t hallB_pin;
  uint16_t hallC_pin;
  // Current sensing
  uint32_t ADC1_trigger;
  uint32_t ADC2_trigger;
  uint32_t ADC3_trigger;
  uint32_t i_phaAB_ch;
  uint32_t i_phaBC_ch;
  uint8_t phaseSwapped;
  uint32_t i_DCLink_ch;
  uint16_t samplingPoint;
  // Model
  RT_MODEL *const rtM;
  ExtU *rtU;
  ExtY *rtY;
  P *rtP;
}motor_t;

void TIM1_Update(void);
void switchADC(void);
void calibration(motor_t *motor);
void runFOC(void);
void runBuzzer(void);
void runMotor(motor_t *motor);
void motorLeft(void);
void motorRight(void);
void enableMotors(uint8_t silent);
void disableMotors(uint8_t silent);

#endif

