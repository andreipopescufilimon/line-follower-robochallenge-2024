/*
  > 1900 - 2000 for homologation
  > 1500 - 1900 for normal runs
*/

#include <Servo.h>

Servo ESC_pin;
#define EDF_PIN 2
#define SIGNAL_PIN 4

void setup() {  
  pinMode(SIGNAL_PIN, INPUT);
  ESC_pin.attach(EDF_PIN);
  
  ESC_pin.writeMicroseconds(1000); // Arm
  delay(1500);
}

void loop() {
  if (digitalRead(SIGNAL_PIN) == HIGH) {
    ESC_pin.writeMicroseconds(1600);
  } else {
    ESC_pin.writeMicroseconds(1000);
  }
}
