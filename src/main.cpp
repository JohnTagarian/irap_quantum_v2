#include <Arduino.h>
#include <base.h>

IMU_t IMU;
Control control;

void setup() {
  Serial.begin(115200);
  initial_motor();
  IMU.initial();

  
}

void loop() {
  control.control_heading(0,90,100);
  // control.move(0,0,800);
  
  // Serial.printf("Yaw : %f\n",IMU.yaw);
  // wheels.move(100,120,0);
  // wheels.cmd_motor(0,0,50);
  // Serial.printf("ENC1: %d , ENC2: %d , ENC3: %d\n",enc_cnt[0],enc_cnt[1],enc_cnt[2]);

}