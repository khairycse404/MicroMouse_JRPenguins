#include "MousePID.h"

MousePID::MousePID(float p, float i, float d, float max_out) {
    kp = p; ki = i; kd = d; max_output = max_out;
    reset();
}

float MousePID::compute(float error, float dt) {
    if (dt <= 0.0f) return 0.0f;
    
    integral += error * dt;
    integral = constrain(integral, -max_output, max_output);
    
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(output, -max_output, max_output);
}

void MousePID::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}