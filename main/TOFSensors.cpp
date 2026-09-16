#include "TOFSensors.h"

// Define new I2C addresses & Pins
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define VL53_ADDRESS 0x29

#define SHT_LOX1 37
#define SHT_LOX2 1
#define SHT_VL53 2

// TOFSensors constructor uses initializer list for VL53L1X pin
TOFSensors::TOFSensors() : lox1(), lox2(), vl53(SHT_VL53) {}

void TOFSensors::begin() {
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_VL53, OUTPUT);

    // Reset sequence
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_VL53, LOW);
    delay(50); 
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    digitalWrite(SHT_VL53, HIGH);
    delay(50);
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_VL53, LOW);
    delay(50);

    // Boot LOX2 FIRST
    digitalWrite(SHT_LOX2, HIGH);
    delay(50); 
    if (!lox2.begin(&Wire)) while (1); 
    lox2.setAddress(LOX2_ADDRESS);
    delay(50);

    // Boot LOX1 SECOND
    digitalWrite(SHT_LOX1, HIGH);
    delay(50);
    if (!lox1.begin(&Wire)) while (1);
    lox1.setAddress(LOX1_ADDRESS);
    delay(50);
    
    // Boot VL53L1X LAST
    digitalWrite(SHT_VL53, HIGH);
    delay(50);
    if (!vl53.begin(VL53_ADDRESS, &Wire)) while (1) delay(10);
    if (!vl53.startRanging()) while (1) delay(10);
    vl53.setTimingBudget(50); 
}

void TOFSensors::update() {
    // --- Read LOX1 (Right Sensor) ---
    // MUST call readRange() first to measure distance and trigger status update!
    uint8_t range_lox1 = lox1.readRange();
    uint8_t status_lox1 = lox1.readRangeStatus();
    
    if (status_lox1 == VL6180X_ERROR_NONE) {
        dist_r_raw = (float)range_lox1;
    } else {
        // Force a large value (>300mm) when out of range or in error state
        // This ensures SIDE_WALL_THRESHOLD triggers correctly!
        dist_r_raw = 300.0f;
    }

    // --- Read LOX2 (Left Sensor) ---
    uint8_t range_lox2 = lox2.readRange();
    uint8_t status_lox2 = lox2.readRangeStatus();
    
    if (status_lox2 == VL6180X_ERROR_NONE) {
        dist_l_raw = (float)range_lox2;
    } else {
        // Force a large value (>300mm) when out of range or in error state
        dist_l_raw = 300.0f;
    }

    // --- Read VL53L1X (Front Sensor) ---
    if (vl53.dataReady()) {
        dist_front = (float)vl53.distance();
        vl53.clearInterrupt();
    }
}

float TOFSensors::getFrontMm()    { return dist_front; }
float TOFSensors::getRightRawMm() { return dist_r_raw; }
float TOFSensors::getLeftRawMm()  { return dist_l_raw; }

// Trigonometric projections for 45-degree mounts (sin/cos 45 = 0.7071)
float TOFSensors::getRightLatMm() { return dist_r_raw * 0.7071f; }
float TOFSensors::getLeftLatMm()  { return dist_l_raw * 0.7071f; }
float TOFSensors::getRightFwdMm() { return dist_r_raw * 0.7071f; }
float TOFSensors::getLeftFwdMm()  { return dist_l_raw * 0.7071f; }