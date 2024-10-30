#include <LF_SData.h>

// Pin Definitions
#define S0 8
#define S1 9
#define S2 10
#define S3 12
#define SIG A0
#define PWM_LEFT 3    // PWM LEFT MOTOR
#define LEFT1 5       // LEFT MOTOR PIN 1
#define LEFT2 4       // LEFT MOTOR PIN 2
#define PWM_RIGHT 11  // PWM RIGHT MOTOR
#define RIGHT1 6      // RIGHT MOTOR PIN 1
#define RIGHT2 7      // RIGHT MOTOR PIN 2

LF_SData sensorData;

// PID Control Variables
float KP = 0.68;                // Proportional constant
float KD = 8;                   // Derivative constant
float KI = 0.002;               // Integral constant
int max_speed = 180;            // Maximum robot speed (0-255)
int forward_brake_speed = 200;  // Forward brake speed
int reverse_brake_speed = 150;  // Reverse brake speed

// PID Calculation Variables
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

int proportional = 0;
int integral = 0;
int derivative = 0;
int differential = 0;
int last_proportional = 0;
int setpoint = 7500;

// Position Variables
long pos = 0, last_pos = 0;

void setup() {
  // Setup PWM frequency for motors
  TCCR2B = TCCR2B & B11111000 | B00000011;  // Set timer 2 divisor to 32 for PWM frequency of 980.39 Hz

  // Pin Mode Setup
  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);

  // Initialize sensor array
  sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

  delay(500);

  // Calibrate sensors
  sensorData.calibrateSensors(false);
}

void loop() {
  while (true) {
    applyBrakes();
    readSensors();
    PIDControl();
  }
}

void readSensors() {
  pos = sensorData.getLinePosition();

  if (last_pos <= 1000 && pos == -1) {
    pos = 0;
  }
  if (last_pos >= 14000 && pos == -1) {
    pos = 15000;
  }

  last_pos = pos;
}

void PIDControl() {
  proportional = pos - setpoint;
  derivative = proportional - last_proportional;
  integral = error1 + error2 + error3 + error4 + error5 + error6;

  last_proportional = proportional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proportional;

  differential = (proportional * KP) + (derivative * KD) + (integral * KI);
  differential = constrain(differential, -max_speed, max_speed);

  if (differential < 0) {
    driveMotors(max_speed, max_speed + differential);
  } else {
    driveMotors(max_speed - differential, max_speed);
  }
}

void applyBrakes() {
  if (pos <= 1500) {
    driveMotors(forward_brake_speed, -reverse_brake_speed);
  }
  if (pos >= 13500) {
    driveMotors(-reverse_brake_speed, forward_brake_speed);
  }
}

void driveMotors(int left_speed, int right_speed) {
  // Left Motor
  if (left_speed >= 0) {
    digitalWrite(LEFT1, HIGH);
    digitalWrite(LEFT2, LOW);
  } else {
    digitalWrite(LEFT1, LOW);
    digitalWrite(LEFT2, HIGH);
    left_speed = -left_speed;
  }
  analogWrite(PWM_LEFT, left_speed);

  // Right Motor
  if (right_speed >= 0) {
    digitalWrite(RIGHT1, HIGH);
    digitalWrite(RIGHT2, LOW);
  } else {
    digitalWrite(RIGHT1, LOW);
    digitalWrite(RIGHT2, HIGH);
    right_speed = -right_speed;
  }
  analogWrite(PWM_RIGHT, right_speed);
}

void stopMotors() {
  driveMotors(0, 0);
}
