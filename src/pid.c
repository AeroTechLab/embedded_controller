#include "pid.h"

/*******************************************************************************
 * LOCAL CONSTANTS
 ******************************************************************************/
const Real SAMPLING_TIME = 1 / (Real)SAMPLING_FREQUENCY;
/*******************************************************************************/

/*******************************************************************************
 * Utility functions
 ******************************************************************************/
void clamp(Real* value, const Real MAX, const Real MIN) {
    if (*value > MAX) {
        *value = MAX;
    } else if (*value < MIN) {
        *value = MIN;
    }
}
/*******************************************************************************/

/*******************************************************************************
 * Error signal
 ******************************************************************************/
Real compute_error(const Real setpoint, const Real measurement) {
    return setpoint - measurement;
}

void update_error(PIDController* pid, const Real error) {
    pid->prev_error = error;
}
/*******************************************************************************/

/*******************************************************************************
 * Measurement signal
 ******************************************************************************/
void update_measurement(PIDController* pid, const Real measurement) {
    pid->prev_measurement = measurement;
}

/*******************************************************************************
 * Proportional
 ******************************************************************************/
void update_proportional_term(PIDController* pid) {
    pid->proportional_term = pid->Kp * pid->prev_error;
}

/*******************************************************************************
 * Integral
 ******************************************************************************/
/* Anti-wind-up via integrator clamping */
void anti_windup(PIDController* pid) {
    clamp(&pid->integrator_term, MAX_INTEGRATOR_LIMIT, MIN_INTEGRATOR_LIMIT);
}

void update_integrator_term(PIDController* pid, const Real error) {
    pid->integrator_term += 0.5f * pid->Ki * pid->T * (error + pid->prev_error);

    anti_windup(pid);
}
/*******************************************************************************/

/*******************************************************************************
 * Derivative (band-limited differentiator)
 *
 * Note: derivative on measurement, therefore minus sign in front of
 * equation!
 ******************************************************************************/
void update_differentiator_term(PIDController* pid, const Real measurement) {
    float measurement_residue = (measurement - pid->prev_measurement);

    pid->differentiator_term =
        -(2.0f * pid->Kd * measurement_residue
          + (2.0f * pid->tau - pid->T) * pid->differentiator_term)
        / (2.0f * pid->tau + pid->T);
}

/*******************************************************************************
 * Compute output and apply limits
 ******************************************************************************/
void update_output(PIDController* pid) {
    pid->out = pid->proportional_term + pid->integrator_term
               + pid->differentiator_term;

    clamp(&pid->out, MAX_OUTPUT, MIN_OUTPUT);
}

/*******************************************************************************
 *  Initialize PID to its first known state
 ******************************************************************************/
void PIDController_Init(PIDController* pid) {
    /* Assign PID gains */
    pid->Ki  = INTEGRAL_GAIN;
    pid->Kd  = DERIVATIVE_GAIN;
    pid->Kp  = PROPORTIONAL_GAIN;
    pid->T   = SAMPLING_TIME;
    pid->tau = DERIVATIVE_LOWPASS_CONSTANT;

    /* Clear controller variables */
    pid->integrator_term     = 0.0f;
    pid->differentiator_term = 0.0f;
    pid->proportional_term   = 0.0f;

    pid->prev_error = 0.0f;

    pid->prev_measurement = 0.0f;

    pid->out = 0.0f;
}

/*******************************************************************************
 *  Update PID state
 ******************************************************************************/
Real PIDController_Update(PIDController* pid, Real setpoint, Real measurement) {
    Real error = compute_error(setpoint, measurement);

    /* update individual terms */
    update_proportional_term(pid);
    update_integrator_term(pid, error);
    update_differentiator_term(pid, measurement);

    update_output(pid);

    /* Store error and measurement for later use */
    update_error(pid, error);
    update_measurement(pid, measurement);

    /* Return controller output */
    return pid->out;
}
