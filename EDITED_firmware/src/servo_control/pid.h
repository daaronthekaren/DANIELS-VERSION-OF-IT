#pragma once

//--- LowpassFilter -----------------------------------------------------------

class LowpassFilter {
  public:
    LowpassFilter();

    void  set_time_constant(float time_constant);
    float update(float value, float dt);
    void  reset(float value);

  private:
    float value_prev;
    float time_constant;
};

//--- PIDController -----------------------------------------------------------

class PIDController {
  public:
    PIDController();
    ~PIDController() = default;

    void  set_parameter(float kP, float kI, float kD, float output_limit, float windup_limit);
    float compute(float error, float dt, float one_over_dt);
    void  reset();

  protected:
    float output_limit; // Maximum output value
    float windup_limit; // Maximum output value

    float kP; // Proportional gain
    float kI; // Integral gain
    float kD; // Derivative gain

    float error_prev; // last tracking error value
    float integral_prev; // last integral component value

    float kI_half; // to avoid multiply
};