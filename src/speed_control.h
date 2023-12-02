#include <Arduino.h>

IntervalTimer speed_control_timer;

// #define MOTOR1_PWM 5
// #define MOTOR1_INA 7
// #define MOTOR1_INB 6

// #define MOTOR2_PWM 2
// #define MOTOR2_INA 3
// #define MOTOR2_INB 4

// #define MOTOR3_PWM 23
// #define MOTOR3_INA 21
// #define MOTOR3_INB 22

#define BUF_L 32
#define BUF_R 31

#define BUF_LIM_FL 15
#define BUF_LIM_FR 16
#define BUF_LIM_SB 14
#define BUF_LIM_SF 17


#define ENCODER1_A 26
#define ENCODER1_B 27

#define ENCODER2_A 24
#define ENCODER2_B 25

#define ENCODER3_A 28
#define ENCODER3_B 29

//
#define MOTOR1_PWM 6
#define MOTOR1_DIR 7

#define MOTOR2_PWM 4
#define MOTOR2_DIR 5

#define MOTOR3_PWM 2
#define MOTOR3_DIR 3



#define MAXIMUM_DUTY 1023

//================ Encoder ================
bool ENC1_A, ENC1_B;
bool ENC2_A, ENC2_B;
bool ENC3_A, ENC3_B;
int pulse1, pulse2, pulse3;

unsigned long prev_time;

String input_string;
int16_t input_speed = 0;

uint32_t count_int_enc[3];
uint32_t pre_count_int_enc[3];

//=========================================
//==               Motor_1               ==
//=========================================
unsigned long long current_time_1;
unsigned long long prev_time_1=0;
unsigned long prev_loop_1=0;
float delta_time_1;
float frequency_1=0, velocity_1=0;
//- Target -
float Target_1 = 0.0f;
//- PID_Related -
float error_1, prev_error_1, integral_1, derivative_1;
//- Parameter -
float Kp1 = 2.42f; // 15.0
float Ki1 = 0.42f; // 0.06
float Kd1 = 0.0f; // 0.5
float control_signal_1 = 0;

//=========================================
//==               Motor_2               ==
//=========================================
unsigned long long current_time_2;
unsigned long long prev_time_2=0;
unsigned long prev_loop_2=0;
float delta_time_2;
float frequency_2=0, velocity_2=0;
//- Target -
float Target_2 = 0.0f;
//- PID_Related -
float error_2, prev_error_2, integral_2, derivative_2;
//- Parameter -
float Kp2 = 2.42f;//15.0;
float Ki2 = 0.42f;//0.06;
float Kd2 = 0.0f;
float control_signal_2 = 0;

//=========================================
//==               Motor_3               ==
//=========================================
unsigned long long current_time_3;
unsigned long long prev_time_3=0;
unsigned long prev_loop_3=0;
float delta_time_3;
float frequency_3=0, velocity_3=0;
//- Target -
float Target_3 = 0.0f;
//- PID_Related -
float error_3, prev_error_3, integral_3, derivative_3;
//- Parameter -
float Kp3 = 2.42f;//10.0;
float Ki3 = 0.42f;//0.05;
float Kd3 = 0.0f;
float control_signal_3 = 0;

volatile long enc_cnt[3];


void Drive_Motor1(int16_t _duty){
  if (_duty > MAXIMUM_DUTY) _duty = MAXIMUM_DUTY;
  else if (_duty < -MAXIMUM_DUTY) _duty = -MAXIMUM_DUTY;

  if (_duty > 0) {
    digitalWrite(MOTOR1_DIR, HIGH);
  }
  else if (_duty < 0) {
    digitalWrite(MOTOR1_DIR, LOW);
  }
  else {
    digitalWrite(MOTOR1_DIR, LOW);
  }
  analogWrite(MOTOR1_PWM, abs(_duty));
}

void Drive_Motor2(int16_t _duty) {
  if (_duty > MAXIMUM_DUTY) _duty = MAXIMUM_DUTY;
  else if (_duty < -MAXIMUM_DUTY) _duty = -MAXIMUM_DUTY;

  if (_duty > 0) {
    digitalWrite(MOTOR2_DIR, HIGH);
  }
  else if (_duty < 0) {
    digitalWrite(MOTOR2_DIR, LOW);
  }
  else {
    digitalWrite(MOTOR2_DIR, LOW);
  }
  analogWrite(MOTOR2_PWM, abs(_duty));
}

void Drive_Motor3(int16_t _duty){
  if (_duty > MAXIMUM_DUTY) _duty = MAXIMUM_DUTY;
  else if (_duty < -MAXIMUM_DUTY) _duty = -MAXIMUM_DUTY;

  if (_duty > 0) {
    digitalWrite(MOTOR3_DIR, HIGH);
  }
  else if (_duty < 0) {
    digitalWrite(MOTOR3_DIR, LOW);
  }
  else {
    digitalWrite(MOTOR3_DIR, LOW);
  }
  analogWrite(MOTOR3_PWM, abs(_duty));
}

void readEncoder1(){
 ENC1_B = digitalRead(ENCODER1_B);
 count_int_enc[0]++;
//  Serial.printf("%d\n",ENC1_B);
 if(ENC1_B > 0){
   enc_cnt[0]++;
 } else{
   enc_cnt[0]--;
 }
  
  current_time_1 = micros();
  delta_time_1 = (float)((current_time_1 - prev_time_1)/1.0e6);
  if(delta_time_1 > 0.019){
    delta_time_1 = 0;
  }
  prev_time_1 = current_time_1;
}

void readEncoder2(){
 ENC2_B = digitalRead(ENCODER2_B);
 count_int_enc[1]++;

 if(ENC2_B > 0){
   enc_cnt[1]++;
 } else{
   enc_cnt[1]--;
 }
  
  current_time_2 = micros();
  delta_time_2 = (float)((current_time_2 - prev_time_2)/1.0e6);
  if(delta_time_2 > 0.019){
    delta_time_2 = 0;
  }
  prev_time_2 = current_time_2;
}

void readEncoder3(){
 ENC3_B = digitalRead(ENCODER3_B);
 count_int_enc[2]++;

 if(ENC3_B > 0){
   enc_cnt[2]++;
 } else{
   enc_cnt[2]--;
 }
  
  current_time_3 = micros();
  delta_time_3 = (float)((current_time_3 - prev_time_3)/1.0e6);
  if(delta_time_3 > 0.019){
    delta_time_3 = 0;  
  }
  prev_time_3 = current_time_3;
}


void speed_control_callback(){
    if(count_int_enc[0] == pre_count_int_enc[0]) delta_time_1 = 0;
    pre_count_int_enc[0] = count_int_enc[0];

    if (delta_time_1 == 0){ //Start
      frequency_1 = 0;
    } else{
      frequency_1 = 1/delta_time_1;  //pulse per sec
    }
    
    velocity_1 = (frequency_1/374.22)*60;  //RPM
  
    //- Kp -
    if(Target_1 == 0) {
      error_1 = 0;
      integral_1 = 0;
      derivative_1 = 0;
      control_signal_1 = 0;
    }
    
    error_1 = abs(Target_1) - velocity_1;
   
    //- Ki -
    integral_1 += error_1;
    if(integral_1 > 16384){
      integral_1 = 16384;
    }
    else if(integral_1 < -16384){
      integral_1 = -16384;
    }
  
    //- Kd -
    derivative_1 = error_1 - prev_error_1;
    prev_error_1 = error_1;
  
    control_signal_1 = (Kp1 * error_1) + (Ki1 * integral_1) + (Kd1 * derivative_1);
    control_signal_1 = constrain(control_signal_1, 0, 1023);

    if(Target_1 < 0) control_signal_1 = -control_signal_1;
  

//-----------------------------------------------------------------
    if(count_int_enc[1] == pre_count_int_enc[1]) delta_time_2 = 0;
    pre_count_int_enc[1] = count_int_enc[1];

    if (delta_time_2 == 0){ //Start
      frequency_2 = 0;
    } else{
      frequency_2 = 1/delta_time_2;  //pulse per sec
    }
    
    velocity_2 = (frequency_2/374.22)*60;  //RPM
  
    //- Kp -
    if(Target_2 == 0) {
      error_2 = 0;
      integral_2 = 0;
      derivative_2 = 0;
      control_signal_2 = 0;
    }
    
    error_2 = abs(Target_2) - velocity_2;
   
    //- Ki -
    integral_2 += error_2;
    if(integral_2 > 16384){
      integral_2 = 16384;
    }
    else if(integral_2 < -16384){
      integral_2 = -16384;
    }
  
    //- Kd -
    derivative_2 = error_2 - prev_error_2;
    prev_error_2 = error_2;
  
    control_signal_2 = (Kp2 * error_2) + (Ki2 * integral_2) + (Kd2 * derivative_2);
    control_signal_2 = constrain(control_signal_2, 0, 1023);

    if(Target_2 < 0) control_signal_2 = -control_signal_2;
  

//-----------------------------------------------------------------
    if(count_int_enc[2] == pre_count_int_enc[2]) delta_time_3 = 0;
    pre_count_int_enc[2] = count_int_enc[2];

    if (delta_time_3 == 0){ //Start
      frequency_3 = 0;
    } else{
      frequency_3 = 1/delta_time_3;  //pulse per sec
    }
    
    velocity_3 = (frequency_3/374.22)*60;  //RPM
  
    //- Kp -
    if(Target_3 == 0) {
      error_3 = 0;
      integral_3 = 0;
      derivative_3 = 0;
      control_signal_3 = 0;
    }
    
    error_3 = abs(Target_3) - velocity_3;
   
    //- Ki -
    integral_3 += error_3;
    if(integral_3 > 16384){
      integral_3 = 16384;
    }
    else if(integral_3 < -16384){
      integral_3 = -16384;
    }
  
    //- Kd -
    derivative_3 = error_3 - prev_error_3;
    prev_error_3 = error_3;
  
    control_signal_3 = (Kp3 * error_3) + (Ki3 * integral_3) + (Kd3 * derivative_3);
    control_signal_3 = constrain(control_signal_3, 0, 1023);

    if(Target_3 < 0) control_signal_3 = -control_signal_3;

    Drive_Motor1(control_signal_1);
    Drive_Motor2(control_signal_2);
    Drive_Motor3(control_signal_3);
}

void initial_base(){
    analogWriteResolution(10);
    analogWriteFrequency(MOTOR1_PWM, 10000);
    analogWriteFrequency(MOTOR2_PWM, 10000);
    analogWriteFrequency(MOTOR3_PWM, 10000);

    pinMode(ENCODER1_A,INPUT);
    pinMode(ENCODER1_B,INPUT);

    pinMode(ENCODER2_A,INPUT);
    pinMode(ENCODER2_B,INPUT);
    
    pinMode(ENCODER3_A,INPUT);
    pinMode(ENCODER3_B,INPUT);


    pinMode(BUF_L,INPUT);
    pinMode(BUF_R,INPUT);

    pinMode(BUF_LIM_FL,INPUT);
    pinMode(BUF_LIM_FR,INPUT);
    pinMode(BUF_LIM_SB,INPUT);
    pinMode(BUF_LIM_SF,INPUT);


    pinMode(MOTOR1_DIR,OUTPUT);
    pinMode(MOTOR1_PWM,OUTPUT);
    pinMode(MOTOR2_DIR,OUTPUT);
    pinMode(MOTOR2_PWM,OUTPUT);
    pinMode(MOTOR3_DIR,OUTPUT);
    pinMode(MOTOR3_PWM,OUTPUT);

    
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_A), readEncoder3, RISING);

    
    speed_control_timer.begin(speed_control_callback,10000);
}


