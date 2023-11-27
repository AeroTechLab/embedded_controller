#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "pid_constants.h"

typedef float Real;

typedef struct {
    Real Kp;
    Real Ki;
    Real Kd;
    Real T;
    Real tau;

    Real prev_measurement;
    Real prev_error;

    Real proportional_term;
    Real integrator_term;
    Real differentiator_term;

    Real out;
} PIDController;

void  PIDController_Init(PIDController* pid);
float PIDController_Update(PIDController* pid, float setpoint,
                           float measurement);

#endif  // !PID_CONTROLLER_H
