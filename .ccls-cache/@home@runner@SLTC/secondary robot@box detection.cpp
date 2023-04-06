#include <NewPing.h>

#define TRIGGER_PIN  23
#define ECHO_PIN     22
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).

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


void loop() {

  delay(50);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

  if (distance==1){
    stop();
    
  }
  else{
    forward();
  }


  
  

}

