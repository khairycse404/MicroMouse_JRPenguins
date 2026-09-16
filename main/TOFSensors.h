#pragma once
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "Adafruit_VL6180X.h"

class TOFSensors {
public:
    TOFSensors();
    void begin();
    void update();
    
    float getFrontMm(); 
    float getRightRawMm();
    float getLeftRawMm();
    float getRightLatMm(); // Calculates lateral distance (sin 45)
    float getLeftLatMm();
    float getRightFwdMm(); // Calculates forward projection (cos 45)
    float getLeftFwdMm();

private:
    Adafruit_VL6180X lox1;
    Adafruit_VL6180X lox2;
    Adafruit_VL53L1X vl53;

    float dist_front = 2000.0f;
    float dist_r_raw = 200.0f;
    float dist_l_raw = 200.0f;
};