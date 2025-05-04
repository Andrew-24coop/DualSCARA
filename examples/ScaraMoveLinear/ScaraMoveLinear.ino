#include <DualSCARA.h>

GStepper2<STEPPER2WIRE> stepper1(6400, 2, 5);
GStepper2<STEPPER2WIRE> stepper2(6400, 3, 6);

DualSCARA scara(74.0, 122.0, stepper1, stepper2);

void setup() {
  Serial.begin(115200);
  stepper1.setAcceleration(3200);
  stepper1.setMaxSpeed(1600);
  stepper2.setAcceleration(3200);
  stepper2.setMaxSpeed(1600);

  scara.moveLinear(100, 50);
}

void loop() {
}