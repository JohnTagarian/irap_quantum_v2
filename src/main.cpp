#include <Arduino.h>
#include <base.h>



unsigned long timer_obs;

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;

IntervalTimer obs_ISR;



void setup() {
  Serial.begin(115200);
  initial_motor();
  IMU.initial();
  delay(1000);

  
}
// unsigned long time_loop_obs;

void plan(void){
  navigation.via_navigate(-0.5,0.0,0,60.0);
  wheels.stop();
  delay(500);
  
  navigation.via_navigate(-0.5,0.51,0,60.0);
  wheels.stop();
  delay(500);

  timer_obs = millis();
  while(1){
    if((control.control_obs_loop(0,90,100) > 50) && (millis()-timer_obs > 2500)){
      break;
    }

  }
  wheels.stop();
  delay(200);
  control.control_heading(0,270,50);
  delay(200);

  // navigation.head_navigate(90);
  // wheels.stop();
  // delay(100);

  navigation.head_navigate(90);
  delay(200);
  navigation.via_navigate(0.0,0.5,90,60,true);
  delay(200);
  while(1){
    if(control.control_slope_loop(90,90,60)){
      break;
    }

  }
  while(1){
    Serial.printf("Stop\n");
    wheels.stop();

  }


}
void loop() {
  // IMU.call_bno();
  // control.reset_time_slope();
  // control.control_slope_loop(0,90,60);

  plan();
  // navigation.head_navigate(90);
  // navigation.via_navigate(0.0,1.0,90,70);
  // navigation.via_navigate(0.0,1.0,90,60);
  // while(1){
  //   Serial.printf("Stop :");
  //   wheels.stop();

  // }
  // while(1){
  //   Serial.printf("Stop \n");
  // }
  // plan();
  
  // wheels.move();
  // while(1){
  //   control.control_heading(0,90,100);

  // }


   
  // while(1){
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