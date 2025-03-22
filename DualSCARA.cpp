// DualSCARA.cpp
#include "DualSCARA.h"

DualSCARA::DualSCARA(float L1, float L2, GStepper2<STEPPER2WIRE> &stepper1, GStepper2<STEPPER2WIRE> &stepper2)
    : l1(L1), l2(L2), stepper1(stepper1), stepper2(stepper2) {}

void DualSCARA::forwardKinematics(float theta1, float theta2, float &x, float &y) {
    theta1 = radians(theta1);
    theta2 = radians(theta2);
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
}

bool DualSCARA::inverseKinematics(float x, float y, float &theta1, float &theta2) {
    float d = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (abs(d) > 1) return false;
    
    theta2 = degrees(acos(d));
    theta1 = degrees(atan2(y, x) - atan2(l2 * sin(radians(theta2)), l1 + l2 * cos(radians(theta2))));
    return true;
}

void DualSCARA::moveTo(float x, float y) {
    float theta1, theta2;
    if (!inverseKinematics(x, y, theta1, theta2)) {
        Serial.println("Target out of reach!");
        return;
    }

    long steps1 = (long)(theta1 * (6400.0 / 360.0));
    long steps2 = (long)(theta2 * (6400.0 / 360.0));

    stepper1.setTarget(steps1);
    stepper2.setTarget(steps2);

    while (!stepper1.ready() || !stepper2.ready()) {
        stepper1.tick();
        stepper2.tick();
    }

    Serial.print("Moved to: X = ");
    Serial.print(x);
    Serial.print(", Y = ");
    Serial.println(y);
}

void DualSCARA::moveLinear(float x1, float y1) {
    float x0, y0, theta1, theta2;
    float theta1_current = stepper1.getCurrent() * (360.0 / 6400.0);
    float theta2_current = stepper2.getCurrent() * (360.0 / 6400.0);
    forwardKinematics(theta1_current, theta2_current, x0, y0);
    
    int steps = 194;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float xt = x0 + t * (x1 - x0);
        float yt = y0 + t * (y1 - y0);
        
        if (!inverseKinematics(xt, yt, theta1, theta2)) continue;
        
        long steps1 = (long)(theta1 * (6400.0 / 360.0));
        long steps2 = (long)(theta2 * (6400.0 / 360.0));

        stepper1.setTarget(steps1);
        stepper2.setTarget(steps2);
        /*
        while (!stepper1.ready() || !stepper2.ready()) {
            stepper1.tick();
            stepper2.tick();
        }
        */
        uint16_t tmr = millis();
        while (millis() - tmr < 50) {
            stepper1.tick();
            stepper2.tick();
        }
    }
    
    Serial.print("Linear move to: X = ");
    Serial.print(x1);
    Serial.print(", Y = ");
    Serial.println(y1);
}

void DualSCARA::moveArc(float x, float y, float i, float j) {
    // Arc movement implementation placeholder
}
