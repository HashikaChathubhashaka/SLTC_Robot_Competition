// Include the necessary libraries
#include <AFMotor.h>  // Library for motor control

// Define motor pins
#define MOTOR_LEFT_DIR 4
#define MOTOR_LEFT_SPEED 5
#define MOTOR_RIGHT_DIR 6
#define MOTOR_RIGHT_SPEED 7

// Create motor objects
AF_DCMotor motorLeft(MOTOR_LEFT_SPEED);
AF_DCMotor motorRight(MOTOR_RIGHT_SPEED);

void setup() {
  // Initialize motor pins
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);
}

void loop() {
  // Make an acute turn to the right
  // Set motor directions for the turn
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);

  // Set motor speeds for the turn
  motorLeft.setSpeed(150);
  motorRight.setSpeed(100);

  // Rotate motors for 1 second (1000 milliseconds)
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
  delay(1000);

  // Stop motors
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  // Resume normal operation
  // Set motor directions for forward motion
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);

  // Set motor speeds for forward motion
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);

  // Continue forward motion
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}