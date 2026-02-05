#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float OutputMin;
  float OutputMax;
  float IntegralLimit;
} PID_Config;

typedef struct {
  float Integrator;
  float PrevError;
  float PrevMeasurement;
  float Output;
} PID_State;

void PID_Init(PID_State *state);
float PID_Calculate(PID_Config *config, PID_State *state, float setpoint,
                    float measurement, float dt);

#endif
