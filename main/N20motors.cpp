#include "N20motors.h"

N20Motor::N20Motor(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB) {
    in1Pin = in1;
    in2Pin = in2;
    encAPin = encA;
    encBPin = encB;
    encoderTicks = 0;
}

void N20Motor::begin() {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    
    pinMode(encAPin, INPUT_PULLUP);
    pinMode(encBPin, INPUT_PULLUP);

    // Initial state: Motor Stopped
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
}

void N20Motor::setSpeed(int speed) {
    // 1. Constrain original input to -255 and 255
    speed = constrain(speed, -255, 255);

    // 2. Map the speed to a safe voltage limit (Max 191 for 6V out of 8V)
    int safeSpeed = map(abs(speed), 0, 255, 0, 190);

    if (speed > 0) { // Forward
        analogWrite(in1Pin, safeSpeed);
        analogWrite(in2Pin, 0); 
    } else if (speed < 0) { // Reverse
        analogWrite(in1Pin, 0); 
        analogWrite(in2Pin, safeSpeed);
    } else { // Stop
        analogWrite(in1Pin, 0);
        analogWrite(in2Pin, 0); 
    }
}

void N20Motor::handleInterrupt() {
    if (digitalRead(encBPin) == HIGH) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

long N20Motor::getTicks() {
    return encoderTicks;
}

void N20Motor::resetTicks() {
    encoderTicks = 0;
}