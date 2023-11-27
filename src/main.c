#include <sys/time.h>

#include "pid.h"

int setitimer(int which, const struct itimerval* newValue,
              struct itimerval* oldValue);
int getitimer(int which, struct itimerval* value);

/*
struct itimerval {
    struct timeval itInterval;  // next value
    struct timeval itValue;     // current value
};

struct timeval {
    long tv_sec;
    long tv_usec;
};
*/

float measurement;
float setpoint;
float control_action;

int main(int argc, char* argv[]) {
    PIDController controller;
    PIDController_Init(&controller);

    // measurement = Sensor_update();
    control_action = PIDController_Update(&controller, setpoint, measurement);

    return 0;
}
