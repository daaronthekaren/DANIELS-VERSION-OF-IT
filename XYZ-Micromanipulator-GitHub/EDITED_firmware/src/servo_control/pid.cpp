#include "pid.h"
#include <algorithm>

PIDController::PIDController()
    : kP(0.0f), kI(0.0f), kD(0.0f), kI_half(0.0f)
    , output_limit(0.0f), windup_limit(0.0f)
    , error_prev(0.0f), integral_prev(0.0f)
{
}

void PIDController::set_parameter(float kP, float kI, float kD, float output_limit, float windup_limit) {
  PIDController::kP = kP;
  PIDController::kI = kI;
  PIDController::kD = kD;
  PIDController::output_limit = output_limit;
  PIDController::windup_limit = windup_limit;

  PIDController::kI_half = kI*0.5f;
}

// PID controller function
float PIDController::compute(float error, float dt, float one_over_dt) {
  // Proportional component
  float proportional = kP * error;
  float output = proportional;

  // Integral component
  if(kI != 0.0f) {
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + kI_half*dt*(error + error_prev);
    integral = std::clamp(integral, -windup_limit, windup_limit);
    output += integral;
    integral_prev = integral;
  }

  // Derivative component
  if(kD != 0.0f) {
    // u_dk = D(ek - ek_1)/Ts
    float derivative = kD*(error - error_prev)*one_over_dt;
    output += derivative;
  }

  // clamp output and store error
  output = std::clamp(output, -output_limit, output_limit);
  error_prev = error;
  
  return output;
}

void PIDController::reset(){
  integral_prev = 0.0f;
  error_prev = 0.0f;
}

//--- LowpassFilter -----------------------------------------------------------

LowpassFilter::LowpassFilter(): value_prev(0.0f), time_constant(1.0f) {
}

void LowpassFilter::set_time_constant(float time_constant) {
  LowpassFilter::time_constant = time_constant;
}

float LowpassFilter::update(float value, float dt) {
  float alpha = time_constant/(time_constant + dt);
  float v = value_prev*alpha + (1.0f - alpha)*value;
  value_prev = v;
  return v;
}

void LowpassFilter::reset(float value) {
  value_prev = value;
}
