//-------------pin difinition-------------------//

//--Motors

//left_Motor
#define LMotorDir1 28        
#define LMotorDir2 26 
#define LMotorEncA 24
#define LMotorPWM 8

//right_Motor
#define RMotorDir1 32  
#define RMotorDir2 34
#define RMotorEncA 30
#define RMotorPWM 7

#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3

volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;


//-------------------constant and variable--------------//
//---motor speeds
int MAX_SPEED = 90;
int MIDDLE_SPEED = 50; //50
int LOWEST_SPEED = 10; 

int left_motor_speed = MIDDLE_SPEED ;
int right_motor_speed = MIDDLE_SPEED ;

  // Increment the number of right wheel pulses
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}
//---------------------Custom Functions------ ------------------//

void reset_pulse_counts() {
  // Reset pulse counts to zero
  right_wheel_pulse_count = 0;
  left_wheel_pulse_count = 0;
}

void turn_left(){
  reset_pulse_counts();
  //encoder_count();

  while(right_wheel_pulse_count <= 200 &&  left_wheel_pulse_count <= 200){
    left();
    set_motor_speed();
    
  }
  forward();

  
}

void turn_right(){
  reset_pulse_counts();
  //encoder_count();

  while(right_wheel_pulse_count <= 200 &&  left_wheel_pulse_count <= 200){
    right();
    set_motor_speed();
    
  }
  forward();

  
}

void u_turn(){
  reset_pulse_counts();
  //encoder_count();

  while(right_wheel_pulse_count <= 395 &&  left_wheel_pulse_count <= 395){
    right();
    set_motor_speed();
    
  }
  forward();

  
}

void encorder_forward(long right_pulse_value , long left_pulse_value ) {
  reset_pulse_counts();
  //encoder_count();

  while(right_wheel_pulse_count <= right_pulse_value &&  left_wheel_pulse_count <= left_pulse_value){
    forward();
    set_motor_speed();
    
  }
  
}

void encorder_backward(long right_pulse_value , long left_pulse_value ) {
  reset_pulse_counts();
  //encoder_count();

  while(right_wheel_pulse_count <= right_pulse_value &&  left_wheel_pulse_count <= left_pulse_value){
    backword();
    set_motor_speed();
    
  }
  
}

void forward() {
  digitalWrite(RMotorDir1, HIGH) ;
  digitalWrite(RMotorDir2, LOW) ;
  digitalWrite(LMotorDir1, HIGH) ;
  digitalWrite(LMotorDir2, LOW) ;
}
void left() {
  digitalWrite(RMotorDir1, HIGH) ;
  digitalWrite(RMotorDir2, LOW) ;
  digitalWrite(LMotorDir1, LOW) ;
  digitalWrite(LMotorDir2, HIGH) ;
}

void backward() {
  digitalWrite(RMotorDir1, LOW) ;
  digitalWrite(RMotorDir2, HIGH) ;
  digitalWrite(LMotorDir1, LOW) ;
  digitalWrite(LMotorDir2, HIGH) ;
}


void right() {
  digitalWrite(RMotorDir1, LOW) ;
  digitalWrite(RMotorDir2, HIGH) ;
  digitalWrite(LMotorDir1, HIGH) ;
  digitalWrite(LMotorDir2, LOW) ;
}

void stop() {
  digitalWrite(RMotorDir1, LOW) ;
  digitalWrite(RMotorDir2, LOW) ;
  digitalWrite(LMotorDir1, LOW) ;
  digitalWrite(LMotorDir2, LOW) ;
}

void set_motor_speed() {
  // Define a speed ratio, e.g., 0.9 for 90% speed of the left motor compared to the right motor
  float speed_ratio = 0.96;
  
  // Calculate the adjusted motor speeds based on the speed ratio
  int adjusted_left_motor_speed = left_motor_speed ;
  int adjusted_right_motor_speed = right_motor_speed * speed_ratio;
  
  // Set the adjusted motor speeds
  analogWrite(RMotorPWM, adjusted_right_motor_speed);
  analogWrite(LMotorPWM, adjusted_left_motor_speed);
}


void setup() {
  Serial.begin(9600);

  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);

//-----------------------pin difinition---------------//

  //pins in motors
  pinMode(LMotorDir1, OUTPUT);
  pinMode(LMotorDir2, OUTPUT);
  pinMode(RMotorDir1, OUTPUT);
  pinMode(RMotorDir2, OUTPUT);
  pinMode(LMotorEncA,OUTPUT);
  pinMode(RMotorEncA,OUTPUT);
  pinMode(LMotorPWM, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);

  //Enabling Left and right motors
  digitalWrite(LMotorEncA,HIGH);
  digitalWrite(RMotorEncA,HIGH);
  set_motor_speed() ;
  forward() ;
  delay(50) ;
}

void loop() {
  forward();
}
