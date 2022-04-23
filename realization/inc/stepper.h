#ifndef STEPPER_H_
#define STEPPER_H_

#include "stdint.h"
#include "stdbool.h"
#include "math.h"

typedef enum {
  STEPPER_STATUS_STOP,
  STEPPER_STATUS_ACCEL,
  STEPPER_STATUS_FREERUN,
  STEPPER_STATUS_FREERUN_PAUSE_FORBIDDEN,
  STEPPER_STATUS_DECEL,
} StepperStatusTypedef;

typedef struct {
	float speed;
	float accel;
	float decel;
	uint32_t stepStart;
	uint32_t stepStop;
	float alpha;
	float tau;
} speedProfileInstance;

typedef struct {
	uint32_t currentStep;
	uint32_t startStep;
	uint32_t stopStep;

	uint32_t accelSteps;
	uint32_t decelSteps;
	uint32_t maxSLimSteps;

	uint32_t savedStepsCurrent;
	uint32_t savedStepsTarget;
	bool pausePending;

	float c0stepAccel;
	float c0stepDecel;

	StepperStatusTypedef status;
	uint8_t id;
} StepperInstance;


void stepperInit(StepperInstance * stepperHandler, uint8_t id);
void stepperCalcSpeedProfile(StepperInstance * stepperHandler, speedProfileInstance * sp);
void stepperUpdateSpeed(StepperInstance * stepperHandler);
StepperStatusTypedef stepperGetStatus(StepperInstance * stepperHandler);

void stepperRun(StepperInstance * stepperHandler);
void stepperStop(StepperInstance * stepperHandler);
void stepperPause(StepperInstance * stepperHandler);

#endif /* STEPPER_H_ */
