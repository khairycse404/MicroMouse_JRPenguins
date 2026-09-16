#pragma once
#include <Arduino.h>

class MousePID {
public:
    MousePID(float p, float i, float d, float max_out);
    float compute(float error, float dt);
    void reset();

private:
    float kp, ki, kd;
    float max_output;
    float integral;
    float prev_error;
};