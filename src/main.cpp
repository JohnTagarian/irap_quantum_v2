#include <Arduino.h>
#include <base.h>

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;

void setup() {
  Serial.begin(115200);
  initial_motor();
  IMU.initial();
  delay(1000);

  
}
float prev_pitch , prev_roll;
float timer_obs;

void loop() {
  // navigation.via_navigate(-0.5,0.0,0,50.0);
  // wheels.stop();
  // delay(500);
  // navigation.via_navigate(-0.5,0.51,0,50.0);
  // wheels.stop();
  // delay(500);
  // // wheels.move(120,90,0);
  while(1){
    IMU.call_bno();
    Serial.printf("diff pitch : %f diff roll :%f\n ",IMU.pitch - prev_pitch,IMU.roll - prev_roll);
    prev_pitch = IMU.pitch;
    control.control_heading(0,90,100);
  
  }
  // control.p2p(0.0,1.0,0);
  // control.control_heading(0,90,100);
  // control.get_distance_vector();
  
  // Serial.printf("Pitch :%f , Roll :%f\n",IMU.pitch,IMU.roll);
  // wheels.move(100,120,0);
  // control.cmd_motor(-50,-50,-50);
  // Serial.printf("ENC1: %d , ENC2: %d , ENC3: %d\n",enc_cnt[0],enc_cnt[1],enc_cnt[2]);

}