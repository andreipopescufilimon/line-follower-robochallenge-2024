/* 
  > LF Enhanced
  > EDF on
*/

#include "LF_SData.h"
#include "xmotionV3.h"

LF_SData lf_sdata;

#define EDF_SIGNAL_PIN 3

#define DISTANCE_PIN A1
#define S0_PIN 1
#define S1_PIN 2
#define S2_PIN 4
#define S3_PIN A5
#define SIG_PIN A4

#define Start A0
#define DipSwitch1 5  // Dipswitch 1 for calibration mode
#define DipSwitch2 6  // Dipswitch 2 for debug mode
#define DipSwitch3 7  // Dipswitch 3 for safe mode

// PID constants
#define KP 0.34 
#define KI 0.04
#define KD 5.4


/*#define KP 0.64 
#define KI 0.08
#define KD 11.4
*/

float previousError = 0;
float integral = 0;

int baseSpeed = 0;
int maxSpeed = 0;

// Variables to store the last six error values for smoother KI calculation
int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
int EDF_init_time = 1000;

int check = 1;

void setup() {
  xmotion.UserLed1(100);

  pinMode(EDF_SIGNAL_PIN, OUTPUT);
  digitalWrite(EDF_SIGNAL_PIN, LOW);

  // Setup pin modes for dip switches and start button
  pinMode(DipSwitch1, INPUT_PULLUP);
  pinMode(DipSwitch2, INPUT_PULLUP);
  pinMode(DipSwitch3, INPUT_PULLUP);
  pinMode(Start, INPUT_PULLUP);

  // Setup sensors and motors
  lf_sdata.setupDistanceSensor(DISTANCE_PIN);
  lf_sdata.setupLineSensors(S0_PIN, S1_PIN, S2_PIN, S3_PIN, SIG_PIN);


  // Determine mode based on dip switch settings
  if (digitalRead(DipSwitch3) == HIGH) {
    // Safe Run mode
    baseSpeed = 40;
    maxSpeed = 55;
  } else if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == HIGH) {
    baseSpeed = 90;
    maxSpeed = 100;
  } else {
    baseSpeed = 45;
    maxSpeed = 70;
  }

  if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == LOW && digitalRead(DipSwitch3) == LOW) {
    // Calibration mode
    unsigned long startTime = millis();
    unsigned long calibrationTime = 5000;  // 5 seconds

    Serial.begin(9600);
    Serial.println("Starting calibration...");

    while (millis() - startTime < calibrationTime) {
      lf_sdata.calibrateSensors(true);
    }
    Serial.println("Calibration complete.");
  } else {
    lf_sdata.calibrateSensors(false);
  }

  xmotion.UserLed2(100);
  xmotion.SETUP();
  xmotion.ToggleLeds(100);

  if (digitalRead(DipSwitch2) == HIGH && digitalRead(DipSwitch1) == LOW && digitalRead(DipSwitch3) == LOW) {
    // Debug mode
    Serial.begin(9600);
  }
}

void loop() {
  if (digitalRead(Start) == HIGH) {

    if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == HIGH) {
      digitalWrite(EDF_SIGNAL_PIN, HIGH);
      if (EDF_init_time > 0) {
        delay(EDF_init_time);
        EDF_init_time = 0;
      }
    }

    int16_t distance = lf_sdata.getDistance();
    if (distance >= 5 && distance <= 300 && check == 1) {
      xmotion.MotorControl(-65, 65);
      delay(150);
      xmotion.MotorControl(70, 70);
      delay(85);
      xmotion.ArcTurn(5, 65, 400);
      xmotion.MotorControl(70, 70);
      delay(30);
      long linePositionC = lf_sdata.getLinePosition();
      while (linePositionC <= 1500 || linePositionC >= 13500) {
        xmotion.MotorControl(60, 60);
        linePositionC = lf_sdata.getLinePosition();
      }

      check = 2;
    }

    long linePosition = lf_sdata.getLinePosition();
    float error = linePosition - 7500;

    // Update integral with last six errors for smoother calculation
    error6 = error5;
    error5 = error4;
    error4 = error3;
    error3 = error2;
    error2 = error1;
    error1 = error;
    integral = error6 + error5 + error4 + error3 + error2 + error1 + error;

    float derivative = error - previousError;
    previousError = error;

    float correction = (KP * error) + (KI * integral) + (KD * derivative);

    int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
    int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

    if (digitalRead(DipSwitch2) == HIGH && digitalRead(DipSwitch1) == LOW && digitalRead(DipSwitch3) == LOW) {
      // Debug mode: print sensor data
      Serial.print("Line Position: ");
      Serial.println(linePosition);
    }

    if (linePosition <= 2500) {
      //cancel_inertia();
      while (linePosition <= 3000) {
        if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == HIGH) {
          xmotion.MotorControl(100, -95);
        } else {
          xmotion.MotorControl(60, -55);
        }
        linePosition = lf_sdata.getLinePosition();
      }
    } else if (linePosition >= 12500) {
      //cancel_inertia();
      while (linePosition >= 12000) {
        if (digitalRead(DipSwitch1) == HIGH && digitalRead(DipSwitch2) == HIGH) {
          xmotion.MotorControl(-95, 100);
        } else { 
          xmotion.MotorControl(-55, 60);
        }
        linePosition = lf_sdata.getLinePosition();
      }
    } else {
      xmotion.MotorControl(map(leftMotorSpeed, 0, 100, 0, 255), map(rightMotorSpeed, 0, 100, 0, 255));
    }
  } else {
    xmotion.MotorControl(0, 0);
    digitalWrite(EDF_SIGNAL_PIN, LOW);
  }
}

void cancel_inertia() {
  xmotion.Backward(70, 5);
}