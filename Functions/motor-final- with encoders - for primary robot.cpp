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

//------------------------------------------//

//------------------turning angle--------------------------------//
#define turn_count_default 705    //for 90 degree
//-------------------------------------------------//
#define countRange 400



//for Encoder counting
volatile unsigned int RMEnCount = 0;
volatile unsigned int LMEnCount = 0;

//pins for encodes
const byte RM_EN_Pin = 3;  
const byte LM_EN_Pin = 2;   

//PID constant for encoders and line following
float previousError;
int enError = 0;
int rightMotorSpeed, leftMotorSpeed, motorSpeed;



//for motors
int minSpeed = 80;
int midSpeed = 100;
int maxSpeed = 120;
float Kp = 0.2; //0.2
float Kd = 0.0001;


//--------------constant for turning-----------

int turn_minSpeed = 70;
int turn_midSpeed = 90;
int turn_maxSpeed = 110;
float t_Kp = 0;
float t_Kd = 0;

//--------------------for PID----//
// float p , d , error ;
// float i = 0 ;
// float prev_error = 0 ;
//---------------------------------//


//-------For line following-------//
int lf_minSpeed = 40;
int lf_midSpeed = 65;
int lf_maxSpeed = 90;

//---constant for line following
float p , d , error ;
float i = 0 ;
float prev_error = 0 ;

float KP = 0.0175 ;
float KD = 0.008 ;
float KI = 0 ;

//-----------------------------------//

//---for IR array
int IR_val[10] = {0,0,0,0,0,0,0,0,0,0} ;
int IR_weight[8] = {-800, -400, -200, -100, 100, 200, 400, 800};


//custom functions

//-------for pin setups
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


//taking IR values............

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

      if(IR_val[0]>150){
        IR_val[0] = 1;
      }else{
        IR_val[0] = 0;
      }

      if(IR_val[1]>150){
        IR_val[1] = 1;
      }else{
        IR_val[1] = 0;
      }

      if(IR_val[2]>150){
        IR_val[2] = 1;
      }else{
        IR_val[2] = 0;
      }

      if(IR_val[3]>150){
        IR_val[3] = 1;
      }else{
        IR_val[3] = 0;
      }

      if(IR_val[4]>150){
        IR_val[4] = 1;
      }else{
        IR_val[4] = 0;
      }

      if(IR_val[5]>150){
        IR_val[5] = 1;
      }else{
        IR_val[5] = 0;
      }

      if(IR_val[6]>150){
        IR_val[6] = 1;
      }else{
        IR_val[6] = 0;
      }
      
      if(IR_val[7]>150){
        IR_val[7] = 1;
      }else{
        IR_val[7] = 0;
      }
      if(IR_val[8]>200){
        IR_val[8] = 1;
      }else{
        IR_val[8] = 0;
      }  

      if(IR_val[9]>200){
        IR_val[9] = 1;
      }else{
        IR_val[9] = 0;
      }       
} 


//-----------PID line Following---------------//
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



void left_90(){
  turnLeft(708);
  Brake(1000);
  
}

void right_90(){
  turnRight(708);
  Brake(1000);
}

void  reverse_turn(){ // 180 degree
  turnRight(1100);
  Brake(1000);
  
}

//-----------------------------------------------------------//


void setup(){
  InitMotors();
  Serial.begin(9600);
  
 


}



//For algorithim
int section=1;
int subsection=1;


void loop(){
  
  if(section==1&& subsection==1){  //line maze


  
  right_90();
  section=2;
  }
  
}