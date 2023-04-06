// Pin definitions for IR sensors
//const int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Pin definitions for motor driver
const int motor1PWM = 2; //5
const int motor1Dir = 3; //4
const int motor2PWM = 5; //2
const int motor2Dir = 4; //3

// Threshold value for detecting black vs white
//const int threshold = 500;

void setup() {
  // Initialize motor driver pins
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  // Initialize IR sensor pins
 // for (int i = 0; i < 8; i++) {
    //pinMode(irPins[i], INPUT);
 // }
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motor1Dir, HIGH); // set direction of left motor
  analogWrite(motor1PWM, leftMotorSpeed); // set speed of left motor
  digitalWrite(motor2Dir, HIGH); // set direction of right motor
  analogWrite(motor2PWM, rightMotorSpeed); // set speed of right motor

}