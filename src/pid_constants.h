#ifndef PID_CONSTANTS_H
#define PID_CONSTANTS_H

//! ------------------------- PID GAINS --------------------------
#define PROPORTIONAL_GAIN 1
#define INTEGRAL_GAIN     1
#define DERIVATIVE_GAIN   1
// ---------------------------------------------------------------

#define SAMPLING_FREQUENCY          1000
#define DERIVATIVE_LOWPASS_CONSTANT 10

// ---------------------------------------------------------------
// Safety boundaries
#define MAX_INTEGRATOR_LIMIT 10
#define MIN_INTEGRATOR_LIMIT -10
#define MAX_OUTPUT           10
#define MIN_OUTPUT           10
// ---------------------------------------------------------------

#endif  // PID_CONSTANTS_H
