// DualSCARA.h
#ifndef DUAL_SCARA_H
#define DUAL_SCARA_H

#include <Arduino.h>
#include <GyverStepper2.h>
#include <math.h>

class DualSCARA {
public:
    DualSCARA(float L1, float L2, GStepper2<STEPPER2WIRE> &stepper1, GStepper2<STEPPER2WIRE> &stepper2);
    
    void forwardKinematics(float theta1, float theta2, float &x, float &y);
    bool inverseKinematics(float x, float y, float &theta1, float &theta2);
    void moveTo(float x, float y);
    void moveLinear(float x1, float y1);
    void moveArc(float x, float y, float i, float j);
    
private:
    float l1, l2;
    GStepper2<STEPPER2WIRE> &stepper1;
    GStepper2<STEPPER2WIRE> &stepper2;
};

#endif