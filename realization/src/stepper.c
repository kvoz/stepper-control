#include "stepper.h"

void stepperInit(StepperInstance * stepperHandler, uint8_t id) {
	StepperInstance stepDrv = {0};

	stepperHandler->id = id;

	stepperHandler->currentStep = 0;
	stepperHandler->startStep = 0;
	stepperHandler->stopStep = 0;

	stepDrv.savedCurrentStep = 0;
	stepDrv.savedStopStep = 0;
	stepDrv.pausePending = false;

	stepperHandler->accelSteps = 0;
	stepperHandler->decelSteps = 0;
	stepperHandler->maxSLimSteps = 0;

	stepperHandler->c0stepAccel = 0.0;
	stepperHandler->c0stepDecel = 0.0;
}

void stepperRun(StepperInstance * stepperHandler) {
	stepperHandler->status = STEPPER_STATUS_ACCEL;

	// 1. Set timer counter to zero
	// 2. Load: ARR <- stepperHandler->c0stepAccel
	// 3. Start timer

}

void stepperStop(StepperInstance * stepperHandler) {
	stepperHandler->status = STEPPER_STATUS_STOP;

	// 1. Stop timer
}

void stepperPause(StepperInstance * stepperHandler) {
	stepperHandler->savedStopStep = stepperHandler->stopStep;
	stepperHandler->stopStep = stepperHandler->currentStep + stepperHandler->decelSteps;
	stepperHandler->savedStepsCurrent = stepperHandler->stopStep;

	stepperHandler->pausePending = true;
}

StepperStatusTypedef stepperGetStatus(StepperInstance * stepperHandler) {
	return stepperHandler->status;
}

void stepperCalcSpeedProfile(StepperInstance * stepperHandler, speedProfileInstance * sp) {
	stepperHandler->startStep = sp->stepStart;
	stepperHandler->stopStep = sp->stepStop + 1;
	stepperHandler->currentStep = sp->stepStart;

	stepperHandler->accelSteps = (sp->stepStop - sp->stepStart) * sp->decel / (sp->accel + sp->decel);
	stepperHandler->maxSLimSteps = sp->speed * sp->speed / (2.0 * sp->alpha * sp->accel);

	if (stepperHandler->maxSLimSteps < stepperHandler->accelSteps) {
		stepperHandler->accelSteps = stepperHandler->maxSLimSteps;
	}

	stepperHandler->decelSteps = stepperHandler->accelSteps * sp->accel / sp->decel;

	stepperHandler->c0stepAccel = sqrtf(2.0 * sp->alpha / sp->accel) / sp->tau;
	stepperHandler->c0stepDecel = sqrtf(2.0 * sp->alpha / sp->decel) / sp->tau;
}

void stepperUpdateSpeed(StepperInstance * stepperHandler) {
	float counter;

	uint32_t currentStep, stopStep;
	currentStep = stepperHandler->currentStep - stepperHandler->startStep;
	stopStep = stepperHandler->stopStep - stepperHandler->startStep;

	if (currentStep < stepperHandler->accelSteps) {
		// accel
		counter = stepperHandler->c0stepAccel * (sqrtf(currentStep + 1) - sqrtf(currentStep));
		stepperHandler->status = STEPPER_STATUS_ACCEL;

		// 1. Load: ARR <- counter
	}
	else if (currentStep >= stopStep - stepperHandler->decelSteps) {
		// decel
		counter = stepperHandler->c0stepDecel * (sqrtf(stopStep - currentStep + 1) - sqrtf(stopStep - currentStep));
		stepperHandler->status = STEPPER_STATUS_DECEL;

		// 1. Load: ARR <- counter
	}
	else {
		// free run
		if (currentStep < stopStep - 2 * stepperHandler->decelSteps - stepperHandler->accelSteps) {
			stepperHandler->status = STEPPER_STATUS_FREERUN;
		} else {
			stepperHandler->status = STEPPER_STATUS_FREERUN_PAUSE_FORBIDDEN;
		}
	}
}
