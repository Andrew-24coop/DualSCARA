// DualSCARA.cpp
#include "DualSCARA.h"

DualSCARA::DualSCARA(float L1, float L2,
    GStepper2<STEPPER2WIRE> &stepper1,
    GStepper2<STEPPER2WIRE> &stepper2,
    LiquidCrystal_I2C &lcd)
: l1(L1), l2(L2), stepper1(stepper1), stepper2(stepper2), _lcd(lcd) {}

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

void DualSCARA::updateDisplay() {
    float theta1 = stepper1.getCurrent() * (360.0 / 6400.0);
    float theta2 = stepper2.getCurrent() * (360.0 / 6400.0);

    float x, y;
    forwardKinematics(theta1, theta2, x, y);

    _lcd.clear();
    _lcd.setCursor(0, 0);
    _lcd.print(theta1);
    _lcd.print("    ");
    _lcd.print(theta2);

    _lcd.setCursor(0, 1);
    _lcd.print("X:");
    _lcd.print(x, 1);
    _lcd.print("  Y:");
    _lcd.print(y, 1);
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
    
    uint16_t tmr = millis();
    while ((!stepper1.ready() || !stepper2.ready()) && (millis() - tmr < 5000)) {
        stepper1.tick();
        stepper2.tick();
    }
    updateDisplay();
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

        updateDisplay();

        uint16_t tmr = millis();
        while (millis() - tmr < 50) {
            stepper1.tick();
            stepper2.tick();
        }
    }
}

void DualSCARA::moveArc(float x, float y, float i, float j, bool dir) {
    float cx = x - i;
    float cy = y - j;
    float r = sqrt(i * i + j * j);
    
    float theta_start = atan2(-j, -i);
    float theta_end = atan2(y - cy, x - cx);
    
    if (dir) {
        if (theta_end < theta_start) theta_end += 2 * PI;
    } else {
        if (theta_start < theta_end) theta_start += 2 * PI;
    }
    
    int steps = 100;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float theta = theta_start + t * (theta_end - theta_start);
        float xt = cx + r * cos(theta);
        float yt = cy + r * sin(theta);
        
        float theta1, theta2;
        if (!inverseKinematics(xt, yt, theta1, theta2)) continue;
        
        long steps1 = (long)(theta1 * (6400.0 / 360.0));
        long steps2 = (long)(theta2 * (6400.0 / 360.0));

        stepper1.setTarget(steps1);
        stepper2.setTarget(steps2);
        
        uint16_t tmr = millis();
        while (millis() - tmr < 50) {
            stepper1.tick();
            stepper2.tick();
        }
    }
}
