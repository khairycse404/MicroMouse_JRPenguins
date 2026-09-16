#pragma once
#include <Arduino.h>

class N20Motor {
private:
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t encAPin;
    uint8_t encBPin;
    volatile long encoderTicks;

public:
    // DRV8833 requires both IN1 and IN2 to be PWM-capable pins
    N20Motor(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB);
    
    void begin();
    void setSpeed(int speed);
    void handleInterrupt();
    long getTicks();
    void resetTicks();
};