#include <Arduino.h>
#include <base.h>


float prev_pitch , prev_roll;
unsigned int timer_obs;

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;

IntervalTimer obs_ISR;
int trig_obs = 0;
int diff_pitch,diff_roll;


float cnt_obs;
bool check_obs = false;

void obs_callback(){
  diff_pitch = fabs(IMU.pitch - prev_pitch);
  diff_roll = fabs(IMU.roll - prev_roll);
  
  if(trig_obs == 1){
  if(diff_pitch > 0.7 || diff_roll > 0.6|| fabs(-3.31 -IMU.pitch) > 3.0 || fabs(-1.625 - IMU.roll) > 3.0){
    Serial.printf("diff pitch : %f diff roll :%f pitch: %f cnt :%f\n ",IMU.pitch - prev_pitch,IMU.roll - prev_roll , IMU.pitch,cnt_obs);
    cnt_obs = 0.0;

  }
  Serial.printf("CNT : %f\n",cnt_obs);
  cnt_obs += 1.0;
  if(cnt_obs > 70.0f){
    check_obs = true;
  }


  
  }
    prev_pitch = IMU.pitch;
    prev_roll = IMU.roll;
}

void setup() {
  Serial.begin(115200);
  initial_motor();
  IMU.initial();
  obs_ISR.begin(obs_callback,10000);
  delay(1000);
  timer_obs = millis();

  
}
// unsigned long time_loop_obs;
void loop() {
  // wheels.move();
  control.control_heading(0,90,100);

    while(1){
    IMU.call_bno();
    trig_obs = 1;
    control.control_heading(0,90,100);
    // wheels.move(90,90,0);
    // wheels.move(100,90+IMU.yaw,300);  
  }
  // while(1){
  //   trig_obs = 0;
  //   Serial.println("Stop");
  //   wheels.stop();
  // }



  // control.p2p(0.0,1.0,0);
  // control.control_heading(0,90,100);
  // control.get_distance_vector();
  
  // IMU.call_bno();
  // Serial.printf("Pitch :%f , Roll :%f\n",IMU.pitch,IMU.roll);

  // -3.31 - > p
  // - 1.625 -> 
  // wheels.move(100,120,0);
  // control.cmd_motor(-50,-50,-50);
  // Serial.printf("ENC1: %d , ENC2: %d , ENC3: %d\n",enc_cnt[0],enc_cnt[1],enc_cnt[2]);

}

void plan(){
  navigation.via_navigate(-0.5,0.0,0,60.0);
  wheels.stop();
  delay(500);
  navigation.via_navigate(-0.5,0.51,0,60.0);
  wheels.stop();
  delay(500);
  while(check_obs == false){
    IMU.call_bno();
    trig_obs = 1;
    control.control_heading(0,90,100);
    // wheels.move(90,90,0);
    // wheels.move(100,90+IMU.yaw,300);  
  }
  while(1){
    trig_obs = 0;
    Serial.println("Stop");
    wheels.stop();
  }

}