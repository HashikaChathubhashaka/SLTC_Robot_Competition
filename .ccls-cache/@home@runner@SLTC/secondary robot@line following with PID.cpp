// Pin definitions for IR sensors
const int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Pin definitions for motor driver

//right motor
const int motor1PWM = 2; 
const int motor1Dir = 3; 

//left motor
const int motor2PWM = 5; 
const int motor2Dir = 4; 

// Threshold value for detecting black vs white
const int threshold = 500;

// PID constants
const float kp = 4.45; // proportional gain
const float ki = 0.0076; // integral gain
const float kd = 1.95; // derivative gain

// Variables for PID control
float lastError = 0.0;
float integral = 0.0;

void setup() {
  // Initialize motor driver pins
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  // Initialize IR sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }
}

void loop() {
  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  // Calculate error value
  float error = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > threshold) {
      error += (i - 3.275); //3.35
    }
  }

  // PID control
  float output = 0.0;
  output += kp * error; // proportional term
  integral += error; // integral term
  output += ki * integral; // integral term
  output += kd * (error - lastError); // derivative term
  lastError = error;

  // Adjust motor speeds based on PID output
  int motorSpeed = 150; // base speed
  int leftMotorSpeed = motorSpeed - output;
  int rightMotorSpeed = motorSpeed + output;

  // Make sure motor speeds are within bounds
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }
  if (leftMotorSpeed > 255) {
    leftMotorSpeed = 255;
  }
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0;
  }
  if (rightMotorSpeed > 255) {
    rightMotorSpeed = 255;
  }

  // Set motor directions and speeds
  digitalWrite(motor1Dir, HIGH); // set direction of left motor
  analogWrite(motor1PWM, leftMotorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, HIGH); // set direction of right motor
  analogWrite(motor2PWM, rightMotorSpeed); // set speed of right motor
}