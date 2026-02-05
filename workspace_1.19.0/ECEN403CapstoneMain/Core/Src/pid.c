#include "pid.h"

void PID_Init(PID_State *state) {
  state->Integrator = 0.0f;
  state->PrevError = 0.0f;
  state->PrevMeasurement = 0.0f;
  state->Output = 0.0f;
}

float PID_Calculate(PID_Config *config, PID_State *state, float setpoint,
                    float measurement, float dt) {
  float error = setpoint - measurement;

  // proportional
  float proportional = config->Kp * error;

  // integral
  state->Integrator += 0.5f * config->Ki * dt * (error + state->PrevError);

  // anti-windup
  if (state->Integrator > config->IntegralLimit) {
    state->Integrator = config->IntegralLimit;
  } else if (state->Integrator < -config->IntegralLimit) {
    state->Integrator = -config->IntegralLimit;
  }

  // derivative (using error for velocity control)
  // Note: derivative-on-measurement would give acceleration for velocity
  // control
  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = config->Kd * (error - state->PrevError) / dt;
  }

  // sum of all terms
  float output = proportional + state->Integrator + derivative;

  // bound output
  if (output > config->OutputMax) {
    output = config->OutputMax;
  } else if (output < config->OutputMin) {
    output = config->OutputMin;
  }

  state->Output = output;
  state->PrevError = error;
  state->PrevMeasurement = measurement;

  return output;
}
