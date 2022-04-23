/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This software component is licensed under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "stepper.h"

// Parameters for speed profile
// steps per round
#define SPR_CARRETE   (200.0F)
// Motor step angle, in rad
#define STEPPER_ALPHA  (2.0F * 3.1415926F / SPR_CARRETE)
// Frequency of handling timers, in Hz
#define STEPPER_TIM_FREQ   (72000.0F)
// Period of handling timers, in seconds
#define STEPPER_TIM_TAU    (1.0F / STEPPER_TIM_FREQ)
// Acceleration radial in rad/s^2
#define STEPPER_ACCEL     10.0
// Acceleration radial in rad/s^2
#define STEPPER_DECEL     10.0
// End of parameters for speed profile

// Cinematic setup
#define STEPPER_PULLEY_DIAMETER  (0.02292F)  // in m
#define STEPPER_PULLEY_RADIUS  (WINDER_CARRETE_PULLEY_DIAMETER / 2.0F) // in m
#define STEPPER_PULLEY_L     (WINDER_CARRETE_PULLEY_DIAMETER * 3.1415926F) // in m
#define STEPPER_STEPS_PER_CM   (SPR_CARRETE / (WINDER_CARRETE_PULLEY_L * 100.0F))
#define STEPPER_STEPS_PER_MM   (SPR_CARRETE / (WINDER_CARRETE_PULLEY_L * 1000.0F))

StepperInstance myStepper;

void Timer_PeriodElapsedCallback();

int main(void)
{

  // Run stepper with parameters of linear speed and distance:
  float linear_speed = 1.0;   // in m/s
  float distance_cm = 100.0;  // in cm

  speedProfileInstance speedProfile = {
    .accel = STEPPER_ACCEL,
    .decel = STEPPER_DECEL,
    .speed = linear_speed / STEPPER_PULLEY_RADIUS;
    .alpha = STEPPER_ALPHA;
    .tau = STEPPER_TIM_TAU;
    .stepStop = lrintf(distance_cm * STEPPER_STEPS_PER_CM);
    .stepStart = 0;
  };

  stepperInit(&myStepper, 0);
  stepperCalcSpeedProfile(&myStepper, &speedProfile);

  // Init timer sheduling stepper motor 

  stepperRun(&myStepper);

  // Run timer sheduling stepper motor in interrupt mode

  while (1)
  {
	  
  }
}

void Timer_PeriodElapsedCallback() {
  if (myStepper.currentStep < myStepper.targetSteps) {
  	myStepper.currentStep++;
  	stepperUpdateSpeed(&myStepper);
  } else {
  	stepperStop(&myStepper);
  }
}