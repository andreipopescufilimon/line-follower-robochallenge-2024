/*
  > LF Turbo Homologation
  > EDF on ~ only this
*/

#include "xmotionV3.h"

#define EDF_SIGNAL_PIN 3

#define Start A0
#define DipSwitch1 5  // Dipswitch 1 for calibration mode
#define DipSwitch2 6  // Dipswitch 2 for debug mode
#define DipSwitch3 7  // Dipswitch 3 for safe mode

int EDF_init_time = 1000;

void setup() {
  xmotion.UserLed1(100);

  pinMode(EDF_SIGNAL_PIN, OUTPUT);
  digitalWrite(EDF_SIGNAL_PIN, LOW);

  // Setup pin modes for dip switches and start button
  pinMode(DipSwitch1, INPUT_PULLUP);
  pinMode(DipSwitch2, INPUT_PULLUP);
  pinMode(DipSwitch3, INPUT_PULLUP);
  pinMode(Start, INPUT_PULLUP);

  xmotion.UserLed2(100);
  xmotion.SETUP();
  xmotion.ToggleLeds(100);
}

void loop() {
  if (digitalRead(Start) == HIGH) {
    digitalWrite(EDF_SIGNAL_PIN, HIGH);
  } else {
    digitalWrite(EDF_SIGNAL_PIN, LOW);
  }
}