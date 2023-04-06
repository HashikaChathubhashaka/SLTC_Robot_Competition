#include <NewPing.h>

#define TRIGGER_PIN  23
#define ECHO_PIN     22
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).


//RGB led pins
int redLED= 25;
int blueLED =27 ;
int greenLED = 29;

// Define color sensor pins
#define S0 9
#define S1 8
#define S2 7
#define S3 6
#define sensorOut 10

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Pin definitions for IR sensors
const int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Pin definitions for motor driver

//right motor-- motor 1
const int motor1PWM = 2; 
const int motor1Dir = 3; 

//left motor -- motor 2
const int motor2PWM = 5; 
const int motor2Dir = 4; 

// Threshold value for detecting black vs white
const int threshold = 500;

// base speed of the motor
int motorSpeed = 150; 

void setup() {

  //pin difinition for LED
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Pulse Width scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  // Set Sensor output as input
  pinMode(sensorOut, INPUT);

  
  Serial.begin(9600);
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

void forward(){
  digitalWrite(motor1Dir, HIGH); // set direction of left motor
  analogWrite(motor1PWM, motorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, HIGH); // set direction of right motor
  analogWrite(motor2PWM, motorSpeed); // set speed of right motor
}

void stop(){
  digitalWrite(motor1Dir, LOW); // set direction of left motor
  analogWrite(motor1PWM, 0); // set speed of left motor
  digitalWrite(motor2Dir, LOW); // set direction of right motor
  analogWrite(motor2PWM, 0); // set speed of right motor
}

// Function to read Red Pulse Widths
int getRedPW() {
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
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
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
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
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}



void loop() {

  delay(50);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  /*
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");
  */

  if (distance==1){
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

  int minimum = min(min(redPW,bluePW),greenPW);
  int average = (redPW+bluePW+greenPW)/3;
  int difference =  abs(minimum - average);


  if(redPW<50 && 50<greenPW  && 50< bluePW){
    
    Serial.print("Red  ");
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, LOW);
    
    
    
  }
  else if(redPW>93 && 93>greenPW  && 93< bluePW ){
    
    Serial.print("green  ");
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, LOW);
    
    
    
  }
  else if(redPW>60 && 60<greenPW  && 62> bluePW){
    
    Serial.print("blue ");
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, HIGH);
    
    
  }else{
    
    Serial.print("white  ");
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
    
    }



  // Print output to Serial Monitor
  Serial.print("Red PW = ");
  Serial.print(redPW);
  Serial.print(" - Green PW = ");
  Serial.print(greenPW);
  Serial.print(" - Blue PW = ");
  Serial.println(bluePW);
    
  }
  else{
    forward();
  }


  
  

}
