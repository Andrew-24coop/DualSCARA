// DualSCARA.h
#ifndef DUAL_SCARA_H
#define DUAL_SCARA_H

#include <Arduino.h>
#include <GyverStepper2.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>   // <<< Added

class DualSCARA {
public:
    DualSCARA(float L1, float L2,
              GStepper2<STEPPER2WIRE> &stepper1,
              GStepper2<STEPPER2WIRE> &stepper2,
              LiquidCrystal_I2C &lcd);  // <<< Added LCD ref to constructor

    void forwardKinematics(float theta1, float theta2, float &x, float &y);
    bool inverseKinematics(float x, float y, float &theta1, float &theta2);
    void moveTo(float x, float y);
    void moveLinear(float x1, float y1);
    void moveArc(float x, float y, float i, float j, bool dir);
    void updateDisplay();  // <<< New function to update display

private:
    float l1, l2;
    GStepper2<STEPPER2WIRE> &stepper1;
    GStepper2<STEPPER2WIRE> &stepper2;
    LiquidCrystal_I2C &_lcd;  // <<< Store LCD reference
};

#endif
