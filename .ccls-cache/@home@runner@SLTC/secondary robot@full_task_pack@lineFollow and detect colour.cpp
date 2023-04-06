#include <NewPing.h>

//---------------- pin difinition  ---------------------//

// pins for untrasonic sensor
#define TRIGGER_PIN 23
#define ECHO_PIN 22
#define MAX_DISTANCE                                                           \
  400 // Maximum distance we want to measure (in centimeters).

// pins for RGB sensor
int redLED = 25;
int blueLED = 27;
int greenLED = 29;

// pins for colour sensor
#define S0 9
#define S1 8
#define S2 7
#define S3 6
#define sensorOut 10

// Pin definitions for motor driver
// right motor-- motor 1
const int motor1PWM = 2;
const int motor1Dir = 3;

// left motor -- motor 2
const int motor2PWM = 5;
const int motor2Dir = 4;

// Pin definitions for IR sensors
const int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

//---------------------------------------------------------------//

//------------------define constant---------------------------------------//

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

// Threshold value for detecting black vs white
const int threshold = 500;

// base speed of the motor
int motorSpeed = 120; // include them inside functions

// PID constants
const float kp = 4.45;   // proportional gain
const float ki = 0.0076; // integral gain
const float kd = 1.95;   // derivative gain

// Variables for PID control
float lastError = 0.0;
float integral = 0.0;

//-----------------------------------------------------------------------------//

//---------------------------create objects-----------------------------//

NewPing sonar(TRIGGER_PIN, ECHO_PIN,
              MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//------------------------------------------------------------------------------//

void setup() {
  // for serial monitor
  Serial.begin(9600);

  //----------------------Outputs -----------------//
  // pin difinition for LED
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Initialize motor driver pins
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  //---------------------------------------------------//

  //----------------------------inputs--------------------//
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);

  // Initialize IR sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

  //------------------------------------------------------//

  // Set Pulse Width scaling to 20% for color sensor
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

//...................custom functions...................................//

//  1.) movements.....................................

void forward() {
  digitalWrite(motor1Dir, HIGH);      // set direction of left motor
  analogWrite(motor1PWM, motorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, HIGH);      // set direction of right motor
  analogWrite(motor2PWM, motorSpeed); // set speed of right motor
}

void stop() {
  digitalWrite(motor1Dir, LOW); // set direction of left motor
  analogWrite(motor1PWM, 0);    // set speed of left motor
  digitalWrite(motor2Dir, LOW); // set direction of right motor
  analogWrite(motor2PWM, 0);    // set speed of right motor
}

void rotate_right(){
  digitalWrite(motor1Dir, LOW); // set direction of left motor
  analogWrite(motor1PWM, motorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, HIGH); // set direction of right motor
  analogWrite(motor2PWM, motorSpeed); // set speed of right motor
}

void rotate_left(){
  digitalWrite(motor1Dir, HIGH); // set direction of left motor
  analogWrite(motor1PWM, motorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, LOW); // set direction of right motor
  analogWrite(motor2PWM, motorSpeed); // set speed of right motor
}


void line_following_pid_forward() {

  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  // Calculate error value
  float error = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > threshold) {
      error += (i - 3.275); // 3.35
    }
  }

  // PID control
  float output = 0.0;
  output += kp * error;               // proportional term
  integral += error;                  // integral term
  output += ki * integral;            // integral term
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
  digitalWrite(motor1Dir, HIGH);           // set direction of left motor
  analogWrite(motor1PWM, leftMotorSpeed);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, rightMotorSpeed); // set speed of right motor
}

//......................................................................

// 2.) color sensor ...............................................

// Function to read Red Pulse Widths
int getRedPW() {
  // Set sensor to read Red only
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
  // Set sensor to read Green only
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
  // Set sensor to read Blue only
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}



int section =1;
int subsection =1 ;

//..................................................................

void loop() {
  if (section ==1 && subsection==1){

  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the
             // shortest delay between pings.

  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print
                                  // result (0 = outside set distance range)
  /*
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");
  */

  //_________after detecting the box_______________
  if (distance == 1) {
    stop();
    // Read Red Pulse Width
    redPW = getRedPW();
    // Delay to stabilize sensor
    delay(200);

    // Read Green Pulse Width
    greenPW = getGreenPW();
    // Delay to stabilize sensor
    delay(200);

    // Read Blue Pulse Width
    bluePW = getBluePW();
    // Delay to stabilize sensor
    delay(200);

    int minimum = min(min(redPW, bluePW), greenPW);
    int average = (redPW + bluePW + greenPW) / 3;
    int difference = abs(minimum - average);

    if (redPW < 50 && bluePW < 50 && greenPW < 50) {

      Serial.print("White  ");
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, HIGH);

    } else if ( redPW > greenPW && greenPW < bluePW) {

      Serial.print("green  ");
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, LOW);

    } else if (bluePW < greenPW && redPW > bluePW) {

      Serial.print("blue ");
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, HIGH);

    } else if (redPW < greenPW && bluePW > redPW) {

      Serial.print("red  ");
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, LOW);
    }

    delay(1000);
    subsection =2 ;
    

  } else {
    line_following_pid_forward();
  }

}

else if (section==1 && subsection==2){
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, LOW);
  
    subsection ==3;
    rotate_left();
    delay(500);
    
  
  
  
}

else if(section==1 && subsection==3){
      line_following_pid_forward();
  
}



}