//secondary

//for ultra sonic sensor
#include <NewPing.h>

//for NRF communication
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//---------------- pin difinition  ---------------------//

// pins for untrasonic sensor
#define TRIGGER_PIN 23
#define ECHO_PIN 22
#define MAX_DISTANCE 200 // Maximum distance we want to measure (in centimeters).

// pins for RGB sensor
int redLED = 19;
int blueLED = 17;
int greenLED = 18;

//pin for normal LED
int normalRed = 31;
int normalBlue = 16;

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

//pin difinition for NRF communication module
RF24 radio(49, 48); 




//---------------------------------------------------------------//

//------------------define constant---------------------------------------//

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

// Threshold value for detecting black vs white
const int threshold = 600;  //500

// base speed of the motor
int motorSpeed = 120; // include them inside functions

// PID constants
const float kp = 7;   // proportional gain
const float ki = 0; // integral gain
const float kd = 0.5;   // derivative gain

// Variables for PID control
float lastError = 0.0;
float integral = 0.0;


//Two addresses to communication
const byte addresses [][6] = {"00001", "00002"}; //One for transmitting and one for receiving

// 00001 - for reading address
// 00002 - for writing address



//for algorithm


int white_bar_count = 0; // 

bool dotted =false;


//-----------------------------------------------------------------------------//

//---------------------------create objects-----------------------------//

NewPing sonar(TRIGGER_PIN, ECHO_PIN,MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//------------------------------------------------------------------------------//

void setup() {
  // for serial monitor
  Serial.begin(9600);

  //for communication
  radio.begin();     

  radio.openWritingPipe(addresses[1]);     //Setting the address to send data
  radio.openReadingPipe(1, addresses[0]);  //Setting the address to recive data
  
  radio.setPALevel(RF24_PA_MIN);           //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.   

  
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
  // Set Sensor output as input - for colour
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


void backward_slow(){
  digitalWrite(motor1Dir, LOW); // set direction of left motor
  analogWrite(motor1PWM, 70); // set speed of left motor
  digitalWrite(motor2Dir, LOW); // set direction of right motor
  analogWrite(motor2PWM, 70); // set speed of right motor
}


void rotate_90_left(){
  stop();
  forward();
  delay(600);
  rotate_left();
  delay(550); 
}

void rotate_90_right(){
  stop();
  forward();
  delay(600);
  rotate_right();
  delay(550); 
}

void rotate_180(){
  stop();
  rotate_left();
  delay(1000);
  
}


//for detecting Junctions -> left
bool left_junction(){

  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }
 
  if(sensorValues[0]<threshold && sensorValues[1]<threshold && sensorValues[2]<threshold){
    return true;
  }
  else{
    return false;
  }
}

//for detecting junctions --> right
bool right_junction(){

    int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }


  if(sensorValues[7]<threshold && sensorValues[6]<threshold && sensorValues[5]<threshold){
    return true;
  }
  else{
    return false;
  }
}

//to detec white bar (left junction + right junction)
bool white_bar(){
     int sensorValues[8];
   for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }
 

  if(sensorValues[0]<threshold && sensorValues[1]<threshold && sensorValues[2]<threshold && sensorValues[3]<threshold && sensorValues[4]<threshold && sensorValues[5]<threshold && sensorValues[6]<threshold && sensorValues[7]<threshold){
    return true;
  }
  else{
    return false;
  }
}



bool black_bar(){
    int sensorValues[8];
   for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  if(sensorValues[0]>threshold && sensorValues[1]>threshold && sensorValues[2]>threshold && sensorValues[3]>threshold && sensorValues[4]>threshold && sensorValues[5]>threshold && sensorValues[6]>threshold && sensorValues[7]>threshold){
    return true;
  }
  else{
    return false;
  }
}


bool side_IR(){//black
    int sensorValues[8];
   for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  if(sensorValues[0]>threshold && sensorValues[7]>threshold){
    return true;
  }
  else{
    return false;
  }
}


void line_following_pid_forward() { //speed 150 (less than normal)

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


void  fast_line_following_pid_forward() { //speed 150 (less than normal)

  int kps  = 30;
  int  kds = 1;
  int kis = 0.5;

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
  output += kps * error;               // proportional term
  integral += error;                  // integral term
  output += kis * integral;            // integral term
  output += kds * (error - lastError); // derivative term
  lastError = error;

  // Adjust motor speeds based on PID output
  int motorSpeed = 110; // base speed
  int leftMotorSpeed = motorSpeed - output;
  int rightMotorSpeed = motorSpeed + output;

  // Make sure motor speeds are within bounds
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }
  if (leftMotorSpeed > 220) {
    leftMotorSpeed = 220;
  }
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0;
  }
  if (rightMotorSpeed > 220) {
    rightMotorSpeed = 220;
  }

  // Set motor directions and speeds
  digitalWrite(motor1Dir, HIGH);           // set direction of left motor
  analogWrite(motor1PWM, leftMotorSpeed);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, rightMotorSpeed); // set speed of right motor
}



void  dotted_line_following() { //speed 150 (less than normal)

  const float kpd = 10;
  const float kdd = 0.7;
  const float kid  = 0;
  

  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  
  //for black area 
  if (sensorValues[0]>threshold && sensorValues[1] >threshold && sensorValues[2]>threshold && sensorValues[3]>threshold && sensorValues[4]>threshold && sensorValues[5]>threshold && sensorValues[6]>threshold && sensorValues[7] > threshold){

   digitalWrite(motor1Dir, HIGH);           // set direction of left motor
  analogWrite(motor1PWM, 207);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, 200); // set speed of right motor

    


    
  }

  //for white lines
  else{
  

  // Calculate error value
  float error = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > threshold) {
      error += (i - 3.275); // 3.35
    }
  }

  // PID control
  float output = 0.0;
  output += kpd * error;               // proportional term
  integral += error;                  // integral term
  output += kid * integral;            // integral term
  output += kdd * (error - lastError); // derivative term
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

}


void  high_dotted_line_following() { //speed 150 (less than normal)

  const float kpd = 20;
  const float kdd = 1;
  const float kid  = 0.3;
  

  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }

  
  //for black area 
  if (sensorValues[0]>threshold && sensorValues[1] >threshold && sensorValues[2]>threshold && sensorValues[3]>threshold && sensorValues[4]>threshold && sensorValues[5]>threshold && sensorValues[6]>threshold && sensorValues[7] > threshold){

   digitalWrite(motor1Dir, HIGH);           // set direction of left motor
  analogWrite(motor1PWM, 207);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, 200); // set speed of right motor

    
    
  }

  //for white lines
  else{
  

  // Calculate error value
  float error = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > threshold) {
      error += (i - 3.275); // 3.35
    }
  }

  // PID control
  float output = 0.0;
  output += kpd * error;               // proportional term
  integral += error;                  // integral term
  output += kid * integral;            // integral term
  output += kdd * (error - lastError); // derivative term
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

}


void  ramp_line_following() { //speed 150 (less than normal)

  const float kpd = 20;
  const float kdd = 0.7;
  const float kid  = 0;
  

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
  output += kpd * error;               // proportional term
  integral += error;                  // integral term
  output += kid * integral;            // integral term
  output += kdd * (error - lastError); // derivative term
  lastError = error;

  // Adjust motor speeds based on PID output
  int motorSpeed = 110; // base speed
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


void PID_ramp_down () { //speed 150 (less than normal)

  const float kpd = 5;
  const float kdd = 0.1;
  const float kid  = 0;
  

  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
  }
  
  //for black area 
  if (sensorValues[0]>threshold && sensorValues[1] >threshold && sensorValues[2]>threshold && sensorValues[3]>threshold && sensorValues[4]>threshold && sensorValues[5]>threshold && sensorValues[6]>threshold && sensorValues[7] > threshold){

   digitalWrite(motor1Dir, HIGH);           // set direction of left motor
  analogWrite(motor1PWM, 207);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, 200); // set speed of right motor

  }
  //for white lines
  else{
  // Calculate error value
  float error = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > threshold) {
      error += (i - 3.275); // 3.35
    }
  }

  // PID control
  float output = 0.0;
  output += kpd * error;               // proportional term
  integral += error;                  // integral term
  output += kid * integral;            // integral term
  output += kdd * (error - lastError); // derivative term
  lastError = error;

  // Adjust motor speeds based on PID output
  int motorSpeed = 210; // base speed
  int leftMotorSpeed = motorSpeed - output;
  int rightMotorSpeed = motorSpeed + output;

  // Make sure motor speeds are within bounds
  if (leftMotorSpeed < 60) {
    leftMotorSpeed = 60;
  }
  if (leftMotorSpeed > 255) {
    leftMotorSpeed = 255;
  }
  if (rightMotorSpeed < 60) {
    rightMotorSpeed = 60;
  }
  if (rightMotorSpeed > 255) {
    rightMotorSpeed = 255;
  }

  // Set motor directions and speeds
  digitalWrite(motor1Dir, HIGH);          // set direction of left motor
  analogWrite(motor1PWM, leftMotorSpeed);  // set speed of left motor
  digitalWrite(motor2Dir, HIGH);           // set direction of right motor
  analogWrite(motor2PWM, rightMotorSpeed); // set speed of right motor
}

}

void slow_pid() { //speed 150 (less than normal)

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
  int motorSpeed = 210; // base speed
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
  analogWrite(motor2PWM, rightMotorSpeed * 0.93); // set speed of right motor
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

//..................................................................

// 3.) Communication.......................................


//for sending a word
void sending(const char* message) {
    delay(5);
  
    // Stop receiving and set the module as transmitter
    radio.stopListening();     

    // Send the message 40 times to reduce errors
    for(int i = 0; i < 40; i++) {
        radio.write(message, strlen(message) + 1);
        delay(20);
    }
}

//outputing the reciving word
String receiving(){
  
  delay(5);
  
  radio.startListening();   //This sets the module as transmitter     

  //reading the incomming massage  
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    //Serial.println(text);
    return String(text);
  }

  return "";
}

//----------------------------------------------------------//

//--------for algorithm-----------//
int section =1;
int subsection =1;

//for box detection colour 
String colour ;
//--------------------//


void loop() {

  if(section==1 && subsection==1){ // transmission

    digitalWrite(normalRed, HIGH);
    String start_massage = receiving();
    delay(500);
    digitalWrite(normalRed, LOW);
    delay(500);
      if (start_massage=="start"){
        
        digitalWrite(normalBlue, HIGH);
        delay(1000);
        digitalWrite(normalBlue, LOW);
        section=2;

      }
}


  else if(section==2 && subsection==1){ // go to the door
    delay(50);
    int distance = sonar.ping_cm();

    if (distance <6 && distance !=0){
      stop();
      subsection=2;
    }

    else{
      line_following_pid_forward();
    }
  }

  else if(section==2 && subsection==2){ //door communication
    String command = receiving();
    if(command =="go1"){

      digitalWrite(normalBlue,HIGH);
      delay(2000);
      digitalWrite(normalBlue,LOW);
      subsection=3;
    }
  }

else if (section ==2 && subsection==3){ //dotted line and colour detection

  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the
             // shortest delay between pings.
  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print
                                  // result (0 = outside set distance range)


  // if (black_bar()==true){
  //   dotted = true;
  // }

  // if (dotted == false){
  // if(right_junction()==true){
  //   stop();
  //   delay(100);
  //   forward();
  //   delay(200);
  // }

  // }

  //_________after detecting the box_______________
  if (distance == 1 && distance !=0) {
    stop();
    digitalWrite(normalRed, HIGH);
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
    digitalWrite(normalRed, LOW);


    int minimum = min(min(redPW, bluePW), greenPW);
    int average = (redPW + bluePW + greenPW) / 3;
    int difference = abs(minimum - average);

    if (redPW < 50 && bluePW < 50 && greenPW < 50) {

      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, HIGH);

      //white communicate 
      digitalWrite(normalRed, HIGH);
      sending("white");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);

      
      digitalWrite(normalRed, HIGH);
      sending("white");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);

      //---------------------

    } else if ( redPW > greenPW && greenPW < bluePW) {

      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, LOW);

      //red communicate 
      digitalWrite(normalRed, HIGH);
      sending("green");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);

      
      digitalWrite(normalRed, HIGH);
      sending("green");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);
      //---------------

    } else if (bluePW < greenPW && redPW > bluePW) {

      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, HIGH);
    
      //blue communicate
      digitalWrite(normalRed, HIGH);
      sending("blue");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);

      
      digitalWrite(normalRed, HIGH);
      sending("blue");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);
      //----------
    
    } else if (redPW < greenPW && bluePW > redPW) {

      Serial.print("red");
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, LOW);

        //red communicate
      digitalWrite(normalRed, HIGH);
      sending("red");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);

      
      digitalWrite(normalRed, HIGH);
      sending("red");
      delay(500);
      digitalWrite(normalRed, LOW);
      delay(500);
      //-------
    }

    delay(500);
    digitalWrite(normalBlue,HIGH);
    delay(1000);
    digitalWrite(normalBlue,LOW);
    subsection =4;
  
  } 
  else {
    high_dotted_line_following();
  }

}

else if (section==2 && subsection==4){ // for rotation to come back
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, LOW);

    
  
    //finish sending
    delay(500);
    rotate_right();
    delay(900);   //for 180

    subsection =5;
  
}

else if(section==2 && subsection==5){  // going to line while finding junction

  int sensorValues[8];
    for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
   }

    if(sensorValues[0]<threshold && sensorValues[1]<threshold && sensorValues[2]<threshold && sensorValues[4]<threshold ){
      forward(); 
      delay(500);
      rotate_left();
      delay(500);
      section=3;
      subsection=1;
    }

      high_dotted_line_following();

      
}


//above thats done.


//ramp up

else if(section==3 && subsection==1){  //go some distace to ramp //cosed this function
  // fast_line_following_pid_forward();
  subsection=2;
}

else if(section==3 && subsection ==2){ // checking the door in ramp
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the
             // shortest delay between pings.

  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print
                                  // result (0 = outside set distance range)
  //check the door again

  if(distance<6 && distance !=0){
    forward();
    delay(80);
    int x = sonar.ping_cm();
    if (distance == x){
    stop();
    subsection=3;
    }
    fast_line_following_pid_forward();
  }

  //ramp up
  fast_line_following_pid_forward();

}

else if(section==3 && subsection==3){ //stop at door and wait for communictaion
  stop();
  digitalWrite(normalRed,HIGH);
  String second_command = receiving();
  if(second_command=="go2"){
  subsection=4;
  digitalWrite(normalRed, LOW);
  }
}
else if(section==3 && subsection==4){  //after communication
  digitalWrite(normalBlue,HIGH);
  delay(500);
  digitalWrite(normalBlue,LOW);
  section=4;
  subsection=1;

}

  
//straight line algorithm
else if (section==4 && subsection==1){ //first 90 junction
    int sensorValues[8];
    for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
   }

    if(left_junction()==true){
      stop();
      delay(2000);
      forward();
      delay(500);
      if(black_bar()==true){
      rotate_left();
      delay(500);
      subsection=2;
      }
      line_following_pid_forward();
    }

  line_following_pid_forward();
  
}

else if (section==4 && subsection==2){ //counting 3 bars to push to box
  if(white_bar_count==3){
    stop();
    delay(1000);
    subsection=3;
  }

  
  if(white_bar() ==true){
    if(side_IR()==true){ // when black is comming one white line is finished.
    white_bar_count++;
    }
  line_following_pid_forward();
  }
  line_following_pid_forward();
}

else if(section==4 && subsection==3){ //pushing the box is finished
  stop();
  //communication
  

  
  rotate_left();
  delay(1000);  //for 180
  section=5;
  subsection=1;
}

//two push button
else if(section==5 && subsection==1){ // rotate 90

    int sensorValues[8];
    for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(irPins[i]);
   }


    if( sensorValues[0]<threshold && sensorValues[1]<threshold && sensorValues[2]<threshold && sensorValues[4]<threshold &&  sensorValues[7]>threshold ){
      stop();
      delay(200);
      forward();
      delay(500);
      stop();
      if(black_bar()==true){
        //correct left junction founded
        stop();
        rotate_90_left();
        //delay(200);
        subsection=2;
        
      }
      line_following_pid_forward();
      
    }
    line_following_pid_forward();
  }

else if(section==5 && subsection==2){
    if(left_junction()==true){
      forward();
      delay(100);
      rotate_90_left();
      subsection=3;
    }
    line_following_pid_forward();
}

else if(section==5 && subsection==3){

  if(left_junction()==true){
    forward();
    delay(50);
    rotate_90_left();
    subsection=4;
    
  }
  dotted_line_following();
}
  
else if (section==5 && subsection==4){
  delay(50);

  int distance = sonar.ping_cm();
  if(distance ==5 && distance !=0){
    stop();
    delay(100);
    forward();
    delay(50);
    rotate_180();
    subsection=5;
  }
  dotted_line_following();
  
}

else if(section==5 && subsection==5){
  delay(50);
  int distance = sonar.ping_cm();
  if (distance==5 && distance !=0){
    stop();
    delay(100);
    forward();
    delay(50);
    rotate_180();
    subsection=6;
  }
  line_following_pid_forward();
}

else if(section==5 && subsection==6){

  if(right_junction()==true){
    forward();
    delay(500);
    rotate_90_right();
    subsection=7;
  }
  line_following_pid_forward();
  
}

else if(section==5 && subsection==7){
    if(white_bar()==true){
      stop();
      delay(200);
      forward();
      delay(1500);
      stop();
      subsection=8;
    }
    
  }

else if(section==5 && subsection==8){
  stop(); //end
  digitalWrite(blueLED, HIGH);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);

  
  
}
  

}