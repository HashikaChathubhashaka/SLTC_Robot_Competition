//primary

//for ultra sonic sensor
#include <NewPing.h>

//for servo motors
#include <Servo.h>

//For communication Module 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//for zig zag path
int c =0;

//flags for encoder line following function
int flag  = 0;

// ################# Pin Difinition ############### //

//---------------Motor pin definition------//

//left_Motor
#define LEFT_FORWARD 26       
#define LEFT_BACKWARD 28 
#define LEFT_MOTOR_ENABLE 24
#define LEFT_MOTOR_PWM 8

//right_Motor
#define RIGHT_FORWARD 32
#define RIGHT_BACKWARD 34
#define RIGHT_MOTOR_ENABLE 30
#define RIGHT_MOTOR_PWM 9 //old 7

//--------------pins for LED bulbs-----------//

//RGB LED
int redLED = 42;
int blueLED = 43;
int greenLED = 41;

//pin for normal LED
int normalRed = 40;

//---------------------sensor pins -------------//

// pins for untrasonic sensor
#define TRIGGER_PIN 38
#define ECHO_PIN 36
#define MAX_DISTANCE 200


//pins for encodes
const byte RM_EN_Pin = 3;  
const byte LM_EN_Pin = 2; 

//colour sensor
#define S0 33
#define S1 31
#define S2 37
#define S3 35
#define sensorOut 39


//------------------NRF pins ------------------------//

RF24 radio(49, 48); // CE, CSN

// ###########################################  //

// ############# Constant and Variables ############## //


// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;


//for ultrasonic distance variable
unsigned int distance;

// for colour 
 String Recived_colour = "red";

//------------------turning angle--------------------------------//
#define turn_count_default 705    //for 90 degree
//-------------------------------------------------//
#define countRange 400

//for Encoder counting
volatile unsigned int RMEnCount = 0;
volatile unsigned int LMEnCount = 0;

//----------encoder movement ---------//

//PID constant  for motor movement
float previousError;
int enError = 0;
int rightMotorSpeed, leftMotorSpeed, motorSpeed;

//for motors Encoder movement
int minSpeed = 80;
int midSpeed = 100;
int maxSpeed = 120;
float Kp = 0.2; //0.2
float Kd = 0.0001;

//-----------------------------------------//

//----------encoder turning--------//

int turn_minSpeed = 70;
int turn_midSpeed = 90;
int turn_maxSpeed = 110;
float t_Kp = 0;
float t_Kd = 0;

//---------------------------------//

//--------------------for PID---------//
// float p , d , error ;
// float i = 0 ;
// float prev_error = 0 ;
//---------------------------------//

//----------For line following------------//
int lf_minSpeed = 45; //40
int lf_midSpeed = 70; //65
int lf_maxSpeed = 95; //90

//---constant for line following
float p , d , error ;
float i = 0 ;
float prev_error = 0 ;

float KP = 0.1 ;  //0.0175
float KD = 0.008 ;  //0.008
float KI = 0 ;  //0
//-----------------------------------//

//---for IR array
int IR_val[10] = {0,0,0,0,0,0,0,0,0,0} ;
int IR_weight[8] = {-800, -400, -200, -100, 100, 200, 400, 800};

//for NRF - addrsses
const byte addresses [][6] = {"00001", "00002"};  //Setting the two addresses. One for transmitting and one for receiving




//############################################################//

// ############# create objects #####################//

// create servo object to control a servo
Servo arm;  
Servo unloard;

//create Ultra sonic Object
NewPing sonar(TRIGGER_PIN, ECHO_PIN,MAX_DISTANCE); // NewPing setup of pins and maximum distance

//#########################################################//


//############## custom functions ############## //  

//-------for pin setups----//
void InitMotors() {
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(RM_EN_Pin, INPUT_PULLUP);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(LM_EN_Pin, INPUT_PULLUP);

  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);

}

//taking IR values............//

void read_junction_IR(){
    IR_val[8] = analogRead(A8) ;     //right junction IR
    IR_val[9] = analogRead(A9) ;     //left  junction IR

    if(IR_val[8]>300){
      IR_val[8] = 1;
    }else{
      IR_val[8] = 0;
      }  

    if(IR_val[9]>300){
      IR_val[9] = 1;
    }else{
      IR_val[9] = 0;
      }       

}

void read_IR(){
   
      IR_val[0] = analogRead(A0) ;        //right panel
      IR_val[1] = analogRead(A1) ;
      IR_val[2] = analogRead(A2) ;
      IR_val[3] = analogRead(A3) ;
      IR_val[4] = analogRead(A4) ;
      IR_val[5] = analogRead(A5) ;
      IR_val[6] = analogRead(A6) ;
      IR_val[7] = analogRead(A7) ;
      IR_val[8] = analogRead(A8) ;     //right junction IR
      IR_val[9] = analogRead(A9) ;     //left  junction IR

      if(IR_val[0]>250){
        IR_val[0] = 1;
      }else{
        IR_val[0] = 0;
      }

      if(IR_val[1]>250){
        IR_val[1] = 1;
      }else{
        IR_val[1] = 0;
      }

      if(IR_val[2]>250){
        IR_val[2] = 1;
      }else{
        IR_val[2] = 0;
      }

      if(IR_val[3]>250){
        IR_val[3] = 1;
      }else{
        IR_val[3] = 0;
      }

      if(IR_val[4]>250){
        IR_val[4] = 1;
      }else{
        IR_val[4] = 0;
      }

      if(IR_val[5]>250){
        IR_val[5] = 1;
      }else{
        IR_val[5] = 0;
      }

      if(IR_val[6]>250){
        IR_val[6] = 1;
      }else{
        IR_val[6] = 0;
      }
      
      if(IR_val[7]>250){
        IR_val[7] = 1;
      }else{
        IR_val[7] = 0;
      }
      if(IR_val[8]>250){
        IR_val[8] = 1;
      }else{
        IR_val[8] = 0;
      }  

      if(IR_val[9]>250){
        IR_val[9] = 1;
      }else{
        IR_val[9] = 0;
      }       
} 

//for colour sensor
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


//for colour detection

bool check_colour(String colour_that_we_want){
  String box_colour;
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

  if (redPW < 50 && bluePW < 50 && greenPW < 50){
    box_colour = "white";
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, HIGH); 
  }

  else if ( redPW > greenPW && greenPW < bluePW) {

    box_colour = "green";
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(redLED, LOW);
}

  else if (bluePW < greenPW && redPW > bluePW) {
    
    box_colour = "blue";
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, LOW);

}
 else if (redPW < greenPW && bluePW > redPW) {
  
    box_colour = "red";
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, LOW);
    digitalWrite(redLED, HIGH);
  } 

  if (box_colour == colour_that_we_want){
    return true;
  }

  return false;
 
}

//-----------PID line Following---------------//

void lineFollowingWithDistance(int firstCount, int secondCount) {

  read_IR();
  error = 0 ;

  for (int i=0; i<=7; i++) {
    error += IR_val[i]*IR_weight[i] ;
    }

  p = error ;
  i = i + error ;
  d = error - prev_error ;
  prev_error = error ;

  float PID_val = (p*KP + i*KI + d*KD);
  int PID_val_int = round(PID_val);

  RMEnCount = 0;
  LMEnCount = 0;
  attachInt();
  
  int totalCount = firstCount + secondCount;

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < totalCount) {

    // calculate line following PID
    rightMotorSpeed = constrain((lf_midSpeed - PID_val_int), lf_minSpeed, lf_maxSpeed);
    leftMotorSpeed = constrain((lf_midSpeed + PID_val_int), lf_minSpeed, lf_maxSpeed);
    
    // calculate encoder PID
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp  + (enError - previousError) * Kd;
    previousError = enError;

    // adjust speed based on distance traveled
    if ((RMEnCount + LMEnCount) / 2 < firstCount) {
      rightMotorSpeed = constrain((rightMotorSpeed - motorSpeed), lf_minSpeed, lf_maxSpeed);
      leftMotorSpeed = constrain((leftMotorSpeed + motorSpeed), lf_minSpeed, lf_maxSpeed);
    } else {
      int decreaseSpeed = map((RMEnCount + LMEnCount) / 2, firstCount, totalCount, lf_midSpeed, lf_minSpeed);
      rightMotorSpeed = constrain((rightMotorSpeed - motorSpeed), lf_minSpeed, 2 * decreaseSpeed - lf_minSpeed);
      leftMotorSpeed = constrain((leftMotorSpeed + motorSpeed), lf_minSpeed, 2 * decreaseSpeed - lf_minSpeed);
    }

    Forward(leftMotorSpeed, rightMotorSpeed);
  }

  detachInt();
}

void lineFollowing() {

  read_IR();
  error = 0 ;

  for (int i=0; i<=7; i++) {
    error += IR_val[i]*IR_weight[i] ;
    }

  p = error ;
  i = i + error ;
  d = error - prev_error ;
  prev_error = error ;

  float PID_val = (p*KP + i*KI + d*KD);
  int PID_val_int = round(PID_val);
 


  rightMotorSpeed = constrain((lf_midSpeed - PID_val_int), lf_minSpeed, lf_maxSpeed);
  leftMotorSpeed = constrain((lf_midSpeed + PID_val_int), lf_minSpeed, lf_maxSpeed);


  Forward(leftMotorSpeed,rightMotorSpeed);
  
}

void high_lineFollowing() {
  KP = 2 ;  //0.0175
  KD = 0.008 ;  //0.008
  KI = 0 ;  //0
  read_IR();

  if (IR_val[0]==1 && IR_val[1]==1 && IR_val[2]==1 && IR_val[3]==1 && IR_val[4]==1 && IR_val[5]==1 && IR_val[6]==1 && IR_val[7]==1){
    Forward(40,40);
  
  }
  else{   
    error = 0 ;

    for (int i=0; i<=7; i++) {


       lf_minSpeed = 30; //40
       lf_midSpeed = 60; //65
       lf_maxSpeed = 100; //90
      error += IR_val[i]*IR_weight[i] ;
      }

    p = error ;
    i = i + error ;
    d = error - prev_error ;
    prev_error = error ;

    float PID_val = (p*KP + i*KI + d*KD);
    int PID_val_int = round(PID_val);


    rightMotorSpeed = constrain((lf_midSpeed - PID_val_int), lf_minSpeed, lf_maxSpeed);
    leftMotorSpeed = constrain((lf_midSpeed + PID_val_int), lf_minSpeed, lf_maxSpeed);
    Forward(leftMotorSpeed,rightMotorSpeed);
  }
  
}



void low_lineFollowing() {
  KP = 0.0175 ;  //0.0175
  KD = 0.01 ;  //0.008
  KI = 0 ;  //0
  read_IR();

     
    error = 0 ;

    for (int i=0; i<=7; i++) {


       lf_minSpeed = 35; //40
       lf_midSpeed = 60; //65
       lf_maxSpeed = 85; //90
      error += IR_val[i]*IR_weight[i] ;
      }

    p = error ;
    i = i + error ;
    d = error - prev_error ;
    prev_error = error ;

    float PID_val = (p*KP + i*KI + d*KD);
    int PID_val_int = round(PID_val);


    rightMotorSpeed = constrain((lf_midSpeed - PID_val_int), lf_minSpeed, lf_maxSpeed);
    leftMotorSpeed = constrain((lf_midSpeed + PID_val_int), lf_minSpeed, lf_maxSpeed);
    Forward(leftMotorSpeed,rightMotorSpeed);
  
  
}


//----------------general movements--------------------------//
void Forward(int rms, int lms) {
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, LOW);
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
  analogWrite(RIGHT_MOTOR_PWM, rms);
  analogWrite(LEFT_MOTOR_PWM, lms);
}

void Backward(int rms, int lms) {
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, HIGH);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, HIGH);
  analogWrite(RIGHT_MOTOR_PWM, rms);
  analogWrite(LEFT_MOTOR_PWM, lms);
}

void Left(int rms, int lms) {
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, HIGH);
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
  analogWrite(RIGHT_MOTOR_PWM, rms);
  analogWrite(LEFT_MOTOR_PWM, lms);
}

void Right(int rms, int lms) {
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, LOW);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, HIGH);
  analogWrite(RIGHT_MOTOR_PWM, rms);
  analogWrite(LEFT_MOTOR_PWM, lms);
}

void PasBrake(int dly) {            //passive Brake
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, HIGH);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, HIGH);
  analogWrite(LEFT_MOTOR_PWM, 0);
  delay(dly);
}

void Brake(int dly) {                    //active brake
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  delay(dly);
}

//-----------for encoders ----------------//
//to get pulses
void attachInt() {
  attachInterrupt(digitalPinToInterrupt(RM_EN_Pin), RM_En_Count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM_EN_Pin), LM_En_Count, CHANGE);
}

//exit from getting pulses
void detachInt() {
  detachInterrupt(digitalPinToInterrupt(RM_EN_Pin));
  detachInterrupt(digitalPinToInterrupt(LM_EN_Pin));
}

void RM_En_Count() {
  ++RMEnCount;
}

void LM_En_Count() {
  ++LMEnCount;
}


//----------------movement by using Encoders---------------//
void goForwardspecificDistance(int firstCount, int secondCount) {
  RMEnCount = 0;
  LMEnCount = 0;

  attachInt();

  int totalCount = firstCount + secondCount;

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < totalCount) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp  + (enError - previousError) * Kd;
    previousError = enError;

    if ((RMEnCount + LMEnCount) / 2 < firstCount) {

      rightMotorSpeed = constrain((midSpeed - motorSpeed), minSpeed, maxSpeed);
      leftMotorSpeed = constrain((midSpeed + motorSpeed), minSpeed, maxSpeed);
    } else {
      //minSpeed = 50;
      int decreaseSpeed = map((RMEnCount + LMEnCount) / 2, firstCount, totalCount, midSpeed, minSpeed);
      rightMotorSpeed = constrain((decreaseSpeed - motorSpeed), minSpeed, 2 * decreaseSpeed - minSpeed);
      leftMotorSpeed = constrain((decreaseSpeed + motorSpeed), minSpeed, 2 * decreaseSpeed - minSpeed);
    }
    Forward(rightMotorSpeed, leftMotorSpeed);
  }
  detachInt();
}

void goBackwardspecificDistance(int firstCount, int secondCount) {
  RMEnCount = 0;
  LMEnCount = 0;

  attachInt();

  int totalCount = firstCount + secondCount;

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < totalCount) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp  + (enError - previousError) * Kd;
    previousError = enError;

    if ((RMEnCount + LMEnCount) / 2 < firstCount) {

      rightMotorSpeed = constrain((midSpeed - motorSpeed), minSpeed, maxSpeed);
      leftMotorSpeed = constrain((midSpeed + motorSpeed), minSpeed, maxSpeed);
    } else {
      //minSpeed = 50;
      int decreaseSpeed = map((RMEnCount + LMEnCount) / 2, firstCount, totalCount, midSpeed, minSpeed);
      rightMotorSpeed = constrain((decreaseSpeed - motorSpeed), minSpeed, 2 * decreaseSpeed - minSpeed);
      leftMotorSpeed = constrain((decreaseSpeed + motorSpeed), minSpeed, 2 * decreaseSpeed - minSpeed);
    }
    Backward(rightMotorSpeed, leftMotorSpeed);
  }
  detachInt();
}

void turnRight(int turn_count) {
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < counts ) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * t_Kp  + (enError - previousError) * t_Kd;
    previousError = enError;

    rightMotorSpeed = constrain((turn_midSpeed - motorSpeed), turn_minSpeed, turn_maxSpeed);
    leftMotorSpeed = constrain((turn_midSpeed + motorSpeed), turn_minSpeed, turn_maxSpeed);

    Right(rightMotorSpeed, leftMotorSpeed);                                                      //change

    if (digitalRead(A4) == 0 && ((RMEnCount + LMEnCount) / 2 > (counts - countRange)) ) {        //A5 , A6 , A7 digitalRead[A5] == 0 &&
      break;
    }
  }
  detachInt();

}

void turnLeft(int turn_count) {
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;
  while ((RMEnCount + LMEnCount) / 2 < counts ) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp  + (enError - previousError) * Kd;
    previousError = enError;

    rightMotorSpeed = constrain((midSpeed - motorSpeed), minSpeed, maxSpeed);
    leftMotorSpeed = constrain((midSpeed + motorSpeed), minSpeed, maxSpeed);

    Left(rightMotorSpeed, leftMotorSpeed);

    if (digitalRead(A3) == 0  &&  (RMEnCount + LMEnCount) / 2 > (counts - countRange) ) {          //A0 , A1 , A2
      break;
    }
  }
  detachInt();
}

void turnRight_from_one_wheel(int turn_count) {  //right wheel back
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < counts) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * t_Kp + (enError - previousError) * t_Kd;
    previousError = enError;

    //rightMotorSpeed = constrain((turn_midSpeed - motorSpeed), turn_minSpeed, turn_maxSpeed);
    rightMotorSpeed = 0;
    leftMotorSpeed = constrain((turn_midSpeed + motorSpeed), turn_minSpeed, turn_maxSpeed);;

    Right(rightMotorSpeed, leftMotorSpeed);  //change

    if (digitalRead(A4) == 0 && ((RMEnCount + LMEnCount) / 2 > (counts - countRange))) {  //A5 , A6 , A7 digitalRead[A5] == 0 &&
      break;
    }
  }
  detachInt();
}

void Right_wheel_forward(int turn_count) {  //right wheel forward
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;

  while ((RMEnCount + LMEnCount) / 2 < counts) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * t_Kp + (enError - previousError) * t_Kd;
    previousError = enError;

    //rightMotorSpeed = constrain((turn_midSpeed - motorSpeed), turn_minSpeed, turn_maxSpeed);
    rightMotorSpeed = 0;
    leftMotorSpeed = constrain((turn_midSpeed + motorSpeed), turn_minSpeed, turn_maxSpeed);;

    Left(rightMotorSpeed, leftMotorSpeed);  //change

    if (digitalRead(A4) == 0 && ((RMEnCount + LMEnCount) / 2 > (counts - countRange))) {  //A5 , A6 , A7 digitalRead[A5] == 0 &&
      break;
    }
  }
  detachInt();
}


void turnLeft_from_one_wheel(int turn_count) { //left_wheel_back
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;
  while ((RMEnCount + LMEnCount) / 2 < counts) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp + (enError - previousError) * Kd;
    previousError = enError;

    rightMotorSpeed = constrain((midSpeed - motorSpeed), minSpeed, maxSpeed);
    leftMotorSpeed = 0;

    Left(rightMotorSpeed, leftMotorSpeed);

    if (digitalRead(A3) == 0 && (RMEnCount + LMEnCount) / 2 > (counts - countRange)) {  //A0 , A1 , A2
      break;
    }
  }
  detachInt();
}

void Left_wheel_forward(int turn_count) { //left_wheel_forward
  RMEnCount = 0;
  LMEnCount = 0;

  int counts = turn_count;

  attachInt();

  previousError = 0;
  while ((RMEnCount + LMEnCount) / 2 < counts) {
    enError = RMEnCount - LMEnCount;
    motorSpeed = enError * Kp + (enError - previousError) * Kd;
    previousError = enError;

    rightMotorSpeed = constrain((midSpeed - motorSpeed), minSpeed, maxSpeed);
    leftMotorSpeed = 0;

    Right(rightMotorSpeed, leftMotorSpeed);

    if (digitalRead(A3) == 0 && (RMEnCount + LMEnCount) / 2 > (counts - countRange)) {  //A0 , A1 , A2
      break;
    }
  }
  detachInt();
}


//.....Turning certain Angles...........//

void right_45(){
  turnRight(500);
  Brake(500);
}

void left_45(){
  turnLeft(500);
  Brake(500); 
}

void left_90(){
  turnLeft(655); //650 good
  Brake(1000);
  
}

void right_90(){
  turnRight(655); //650 good
  Brake(1000);
}

void  reverse_turn(){ // 180 degree

  // Left_wheel_forward(330);
  // Brake(1000);

  // turnRight_from_one_wheel(330);
  // Brake(1000);

  turnRight(1050);
  Brake(200);

  
  
}

//--------------------------------------//

//-------Servo Motor Control-------------//

void unload(){
  
  int pos;
  for (pos = 180; pos >= 60; pos -=1) {  // loop from 180 degrees to 60 degrees in 5 steps
    unloard.write(pos);  // tell servo to go to position
    delay(20);  // wait for servo to reach position
    }
}

void getload(){
  int pos;
  for (pos = 40; pos <= 180; pos +=1) {  
    unloard.write(pos);  // tell servo to go to position
    delay(1);  // wait for servo to reach position
  }


}




//-------------------------------------------------------------//
//encoders enable
void  encoder_enable(){

      //Enable Encoders
      RMEnCount = 0;
      LMEnCount = 0;
      attachInt(); 
}
    

//-------------communication---------------//

//for sending a word
void sending(const char* message) {
    digitalWrite(normalRed,HIGH);
    delay(5);
  
    // Stop receiving and set the module as transmitter
    radio.stopListening();     

    // Send the message five times to reduce errors
    for(int i = 0; i < 20; i++) {
        radio.write(message, strlen(message) + 1);
        delay(20);
    }
        digitalWrite(normalRed,LOW);
        delay(100);
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

//---------------------------------------------------------//

void led_blink(int bulb_pin){
  digitalWrite(bulb_pin,HIGH);
  delay(1000);
  digitalWrite(bulb_pin,LOW);
  delay(1000);
}

//-----------------------------------------------------------//


void grab_arm(){
  arm.write(0);   // open position of arm
  delay(1000);         // waits for a second
  arm.write(50);   // close position of arm
  delay(2000); 
  
}

void release_arm(){
  arm.write(50);   // close position
  delay(1000);         // waits for a second
  arm.write(0);   // open position
  delay(2000); 
  
}


//#######################################################//



void setup(){
  InitMotors();
  Serial.begin(9600);

  //servo pins
  arm.attach(10);  
  unloard.attach(11);


  //initial positions of arm and unloarding 
  unloard.write(180); // for unloarding door
  arm.write(0); //for arm


  //for radio communicaiton
  radio.begin();                           //Starting the radio communication
  radio.openWritingPipe(addresses[0]);     //Setting the address at which we will send the data
  radio.openReadingPipe(0, addresses[1]);  //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);           //You can set it as minimum or maximum depending on the distance between the transmitter and receiver. 

//---------------for Colour sensor ---------------------//  
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

  

}
  


//-------------For algorithim------------//
int section=1;
int subsection=1;

bool box_found = false;

//---------------------------------------//

//function for go line following with encoder count
void encoder_line_following(int distance,int next_subsection){
  if(flag==0){
    encoder_enable();
    flag++;
  }
  else{
    
    if(RMEnCount>distance){
      Brake(1000);
      detachInt();
      Brake(100);
      flag=0;
      subsection=next_subsection;
    }
    else{
      lineFollowing();
    }
  }  
}


void loop(){
  
if(section==1&& subsection==1){  //line maze
 
  read_IR();
    
  if ((IR_val[8] == 0&& IR_val[9]==1) || (IR_val[8]==0 && IR_val[9]==0) ){
    goForwardspecificDistance(100, 100);
       //white box
       
    read_IR();
    if (IR_val[8] == 0){
    PasBrake(500);
    section=2; //checkpoint 1
    }
         
    else{
      right_90();
      Brake(500);
      goBackwardspecificDistance(90,90);
      Brake(500);
      
    }
  }

//go straight(juncton with forward and left)
    else if (IR_val[9] == 0 && IR_val[8]==1 ){
      //Brake(500);
        goForwardspecificDistance(100, 100);
        Brake(500);
       //white box
       read_IR();
     
       if (IR_val[9] == 0 && IR_val[6] == 0){
          PasBrake(1000);
          section=2;   //checkpoint 1
       }
      else if(IR_val[3] == 0 || IR_val[4] == 0 || IR_val[5] == 0){
          lineFollowing();
      }
      else {
        left_90();
        Brake(500);
        goBackwardspecificDistance(90,90);
        Brake(500);
      }
}

//dead end
else if(IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 && IR_val[8] == 1 && IR_val[9] == 1) {
  goForwardspecificDistance(100, 100);
  read_IR();
  if(IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 && IR_val[8] == 1 && IR_val[9] == 1) {
  Brake(500);    
  reverse_turn();
  Brake(500);
  goBackwardspecificDistance(90,90);
  Brake(500);
  }
}

else{
  lineFollowing();
  
}
  }

else if(section==2 && subsection==1){  //enter to the white box
    goForwardspecificDistance(100, 100);
    Brake(1000);
    right_90();
    PasBrake(1000);

    //Enable Encoders
    encoder_enable();
    subsection =2;

}
else if(section==2 && subsection==2){ 

    if(RMEnCount>700){
      Brake(2000);
      detachInt();
      reverse_turn();
      Brake(100);

      encoder_enable();
      subsection=3;
    }
    else{
      lineFollowing();
    }
   // Brake(1000);

  //subsection=10;

}
else if(section==2 && subsection==3){  // unloading secondary robot

    if(RMEnCount >400){
      Brake(100);

      detachInt();
      unload();
      Brake(2000);

      sending("start");
      delay(1000);


      sending("start");
      delay(1000);


      //delay the time for go to secondary robot
      led_blink(redLED); //2s delay
      led_blink(redLED);
      led_blink(greenLED);

      subsection=4;
    }
    else{
        lineFollowing();
  }  
}
else if(section==2 && subsection==4){ //close the gate 


  getload();
  delay(1000);
  goForwardspecificDistance(150, 150);
  Brake(1000);
  right_90();

  subsection=5;


}

else if(section==2 && subsection==5){

  //speeds for after unloarding secondary robot

   minSpeed = 80; //80
   midSpeed = 100; //100
   maxSpeed = 120; //120

   turn_minSpeed = 70;
   turn_midSpeed = 90;
   turn_maxSpeed = 110;

   lf_minSpeed = 30; //40
   lf_midSpeed = 55; //65
   lf_maxSpeed = 80; //90
   KP = 0.2;

  
  

  read_junction_IR();
  if (IR_val[8]==0){  //turn right from the first junction after the check box 1
    Brake(200);
    goForwardspecificDistance(85,85);
    Brake(500);
    turnRight(650);
    Brake(500);
    goBackwardspecificDistance(110, 110);
    Brake(500);
    subsection=6;

  }

  else{
    lineFollowing();
  }  
  
}
else if(section==2 && subsection ==6){
  read_IR();
   
  if ((IR_val[3]==1 && IR_val[4]==1)&&(IR_val[0]==0 && IR_val[7]==0)){  //detect y junction before box detection
    Brake(500);
    section=3;
    subsection=1; 

  }

  else{
    lineFollowing();
  }

}

else if(section==3 && subsection ==1){   // turn 45 at y junction
  right_45();
  subsection =2;

}

else if(section==3 && subsection==2){ // check the box at right path
  
  
  int distance = sonar.ping_cm(); 
  delay(100);


  if (distance <6 && distance !=0){   // check distance to the box at right path
    Brake(1000);
    subsection =10;
    
    
  }
  else{
  lineFollowing();
  }
}

else if(section==3 && subsection==10){ //check again the box at right path 

    int dis =100;
    delay(100);
    dis = sonar.ping_cm();
    delay(100);
    if(dis<6){  
    Brake(200);  //box was detected at right path
    subsection = 3;
    
    }
   else{
    

    // read_junction_IR();   
    // if (IR_val[8]==0){  
    //   Brake(2000);
    //   goForwardspecificDistance(50,50);
    //   Brake(200);
    //   right_45();
    //   Brake(200);
    //   section=4;
    //   subsection=1;

    // }

  low_lineFollowing();
  }

}

else if (section==3 && subsection==3){ // go from right to left path
    goBackwardspecificDistance(40, 40);
    Brake(100);
    turnLeft(330);
    Brake(100);
    goForwardspecificDistance(110,110);//want to find
    Brake(200);
    turnRight(270);
    Brake(1000);
    subsection=4;
    
  }

else if (section==3 && subsection==4){   
  
  int distance=100;    //find the distance to the box at left path
  delay(100);
  distance = sonar.ping_cm();
  delay(100);


  if (distance <6 && distance != 0){  //check for the box on left path
    Brake(1000);
    // goBackwardspecificDistance(40, 40);
    // Brake(500);
    // goForwardspecificDistance(20, 20);    
    // Brake(500);
    subsection =5;
    
    
  }
  else{
 

    read_junction_IR();   
    if (IR_val[8]==0){  // identify y junction from left path
      Brake(2000);
      goForwardspecificDistance(50, 50);
      section=4;
      subsection=1;

    }

  low_lineFollowing();
  }
    
  }
else if (section==3 && subsection==5){   // check for box again on left path

    int d=100;   
    delay(100);
    d = sonar.ping_cm();
    delay(100);
    if(d<8 && d !=0){    //box was detected
    subsection = 7;
    
    }
   else{
   

    read_junction_IR();   
    if (IR_val[8]==0){  // identify y junction from left path
      Brake(2000);
      goForwardspecificDistance(50, 50);
      section=4;
      subsection=1;

    }

  low_lineFollowing();
  }
  }



else if (section==3 && subsection==7){   // go from left path to right path
    Brake(200);
    goBackwardspecificDistance(40, 40);
    Brake(100);
    turnRight(330);
    Brake(100);
    goForwardspecificDistance(100,100);
    Brake(200);
    turnLeft(320);
    Brake(200);
    section = 3;
    subsection= 11;
  }

else if (section==3 && subsection==11){  //identify y junction at right path
  if(IR_val[9]==0 && IR_val[7]==0 && IR_val[8]==1){
    Brake(100);
    goForwardspecificDistance(100, 100);
    turnRight(40);
    Brake(500);
    section = 4;
    subsection= 1;
    }
    high_lineFollowing();
  

}
  // else if (section==3 && subsection==6){    // identify y junction from right path

  // high_lineFollowing();

    // read_junction_IR();
    // if (IR_val[9]==0){

    //   //for right path
    //   Brake(2000);
    //   goForwardspecificDistance(50,50);
    //   Brake(200);
    //   left_45();
    //   Brake(2000);
    //   section=4;
    //   subsection=1;
    // }
    //   lineFollowing();
      
    // }

else if(section==4 && subsection==1){  //go to check box 2 from y junction

  
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0 ){
    Brake(200); //200
    goForwardspecificDistance(100,100);
    Brake(2000);
    
    digitalWrite(redLED,HIGH);
    sending("go1");
    delay(500);
    digitalWrite(redLED,LOW);
    
    digitalWrite(redLED,HIGH);
    sending("go1");
    delay(500);
    digitalWrite(redLED,LOW);
    subsection=2;
    
  }
  high_lineFollowing();  //dotted line following before check box 2 

}

else if (section==4 && subsection==2){
  Brake(100);
  
  Recived_colour = receiving();
  if(Recived_colour == "red"){
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, LOW);
      subsection=3;
    
  }
  else if (Recived_colour=="green"){
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, LOW);
      subsection=3;
    
  }

  else if (Recived_colour =="blue"){
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      digitalWrite(blueLED, HIGH);
      subsection=3;
    
  }

  else if(Recived_colour =="white"){
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, HIGH);
      digitalWrite(blueLED, HIGH);


      subsection=3;
  }
}

else if(section==4 && subsection ==3){  //turn right from the first junction after the checkbox 2

    digitalWrite(redLED, LOW);
    digitalWrite(greenLED,LOW);
    digitalWrite(blueLED, LOW);

    read_junction_IR();
    if(IR_val[8]==0){
    Brake(300);
    goForwardspecificDistance(100, 100);
    Brake(300);
    right_90();
    Brake(300); 
    section=5;  //section 5 --> Box finding and grabing 
    subsection=1;

  }

  else{
    lineFollowing();
  }

}

else if(section==5 && subsection==1){   // check the box is founded or not
  if (box_found == true){ //correct box was founded
    subsection=2;
    
  }

  else{
    read_junction_IR();
    
    if(IR_val[8]==0){  //turn to check the box colour

        Brake(200);

        goForwardspecificDistance(270,270);
        Brake(200);
        turnRight_from_one_wheel(330);
        Brake(300);
        subsection=3;
    }
    else{
      lineFollowing();
    }
  }
}

else if (section==5 && subsection==2){ //box is founded
  read_IR();

  if(IR_val[9] == 0 ){ //avoid the path to left on the curved path before hatching bax
    Brake(500);
    turnRight(90); 
    Brake(500);
    goForwardspecificDistance(50, 50);
    section = 5;
    subsection = 5;
  }

else{
  high_lineFollowing();
}
}

else if (section==5 && subsection ==3){ //go to the box
      int distance = sonar.ping_cm();
      if(distance<3 && distance != 0){
        Brake(500);
        goForwardspecificDistance(8,8);
        Brake(500);
        subsection=4;
      }
      else{
        lineFollowing();
      }
  
}

else if(section==5 && subsection==4){    //check the colour of the box
  if(check_colour(Recived_colour)==true){      //want to give reciving colour
    box_found=true;
    grab_arm();    // grab the box
    
  }
  else{
    box_found==false;
  }

  goBackwardspecificDistance(20, 20);
  turnLeft(650);
  subsection=1;
  
}

else if(section==5 && subsection==5){  //box hatching

  read_IR(); 

  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0 ) {
    Brake(500);
    goForwardspecificDistance(30, 30);
    Brake(500);
    release_arm();
    Brake(500);
    goBackwardspecificDistance(70, 70);
    Brake(500);
    reverse_turn();
    Brake(500);
    goBackwardspecificDistance(100, 100);
    Brake(500);
    section = 6;
    subsection = 1;
} 
 
else{
  high_lineFollowing();
}
}
  
else if(section==6 && subsection==1){  //stay after hatching the box
     Brake(100);
     
     //transmitt
    digitalWrite(normalRed,HIGH);
    sending("go2");
    delay(500);
    digitalWrite(normalRed,LOW);

    digitalWrite(normalRed,HIGH);
    sending("go2");
    delay(500);
    digitalWrite(normalRed,LOW);
  

    section =6;
    subsection = 2;
    }

else if(section==6 && subsection==2){
  String mg = receiving();
  if (mg == "go3"){
    section=7;
    subsection=1;
  }
  
}

else if(section==7 && subsection==1){  //after transmission go forward and turn right from the first junction
    if(IR_val[8] == 0){
      Brake(500);
      turnRight(500);
      Brake(500);
      subsection = 2;
     }
    else{
      high_lineFollowing();
    }  

}

else if(section==7 && subsection==2){  // check the four way junction                    
  if (IR_val[8] == 0 && IR_val[9] == 0){
      Brake(1000);
      goForwardspecificDistance(70,70);
      subsection==3;    
  }
  else{
    high_lineFollowing();
  }
}

else if(section==7 && subsection==3){  // avoid the four way junction and turn left to detect y junction
  if (IR_val[9] == 0){
    Brake(500);
    goForwardspecificDistance(100,100);
    Brake(500);
    turnLeft(670);
    Brake(500);
    subsection = 4;
  }
  else{
    high_lineFollowing();
  }
}

else if(section==7 && subsection==4){  //detect y junction and choose zig zag path
  read_IR();
  if (IR_val[3]==1 && IR_val[4]==1&&IR_val[0]==0 && IR_val[7]==0){
    Brake(500);
    right_45();
    Brake(500);
    section=8;
    subsection=1; 

  }
  else{
    lineFollowing();
  }

}

  else if(section == 8 && subsection == 1){  //enter to the zig zag
        read_IR();
        if(IR_val[9] == 0 && c==0){ // turn left
          c=1;
          goForwardspecificDistance(100, 100);
          turnLeft(650);
          goForwardspecificDistance(150, 150);
          
          }

        else  if(IR_val[8] == 0 && c==1 ){ //  turn right
           c=0;
           goForwardspecificDistance(100, 100);  
           turnRight(650);
           goForwardspecificDistance(150, 150);
          }  
            
      else  if(IR_val[9] == 0 && c==1 ) { // leaving from the zig zag
           goForwardspecificDistance(100, 100);  
           turnRight(350); 
           Brake(5000);
           section = 9;
           subsection = 1;
           
        
        }
     else {
          
       lineFollowing();  
      }
           
        
        }

     if( section == 9 &&  subsection ==1){ //  go to the finish check box
       read_IR();
      if( IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0 && IR_val[8] == 0 && IR_val[9] == 0){
        Brake(1000);
        goForwardspecificDistance(200, 200);  
        Brake(100000);
        }
      else{
         lineFollowing();  
        }  
      
      
      }


}