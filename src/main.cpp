#include <Arduino.h>
#include <base.h>

float path_x[20] = {-0.025, -0.05, -0.07500000000000001, -0.1, -0.125, -0.15};
float path_y[20] = {0.0022, 0.0091, 0.0215, 0.0414, 0.075, 0.1414};
float npath = 6;



unsigned long timer_obs;

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;

IntervalTimer obs_ISR;

// unsigned long time_loop_obs;

void plan(void){
  navigation.via_navigate(-0.52,0.0,0,60.0);
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

  // delay(100);
  // navigation.via_navigate(0.0,0.3,90,60,true);
  // delay(200);
  // navigation.via_navigate(-0.43,0.3,90,60);
  // wheels.stop();
  // delay(200);
  // navigation.via_navigate(0.0,0.4,90,60,true);
  // wheels.stop();
  // delay(200);
  while(1){
    Serial.printf("Stop\n");
    wheels.stop();

  }


}
void step_back(int target_angle){
  while(1){
    control.control_heading(target_angle,270,60);
    if(analogRead(BUF_L) > 300 && analogRead(BUF_R) > 200){
      control.control_heading(target_angle,270,60);
      delay(100);
      break;

    }
  }
}

void set_origin(int target_angle){
  while(1){
    if(analogRead(BUF_L) < 300 && analogRead(BUF_R) < 200){
      step_back(target_angle);
      break;

    }
    else{
      control.control_heading(target_angle,90,60);
    }

  }
}

void plan2(void){
  navigation.via_navigate(0.51,0.0,0,80);
  wheels.stop();
  delay(200);

  navigation.via_navigate(0.5,-0.52,0,80);
  navigation.head_navigate(180);
  wheels.stop();
  delay(200);

  timer_obs = millis();
  while(1){
    if((control.control_obs_loop(180,90,100) > 50) && (millis()-timer_obs > 2500)){
      break;
    }

  
  }
  wheels.stop();
  delay(200);
  set_origin(180);
  navigation.head_navigate(270);
  delay(200);
  navigation.via_navigate(-0.02,-0.5,270,100,true);
  delay(200);
  while(1){
    if(control.control_slope_loop(270,90,60)){
      break;
    }

  }
  set_origin(270);
  navigation.head_navigate(0);
  set_origin(0);
  navigation.head_navigate(270);
  set_origin(270);
  navigation.via_navigate(-0.3,0.0,270,120,true);
  navigation.via_navigate(-0.45,-0.1,270,120);
  navigation.via_navigate(-0.5,-0.8,270,120);
  navigation.via_navigate(0.2,-0.2,270,120,true);
  navigation.via_navigate(0.25,0.0,270,120,true);
  navigation.via_navigate(0.15,0.15,270,120,true);
  navigation.via_navigate(0.15,0.15,270,120,true);
   navigation.via_navigate(0.0,0.0,270,120,true);
  navigation.p2p_navigate(0.0,0.4,270);


  // navigation.head_navigate(270);
  // wheels.stop();
  // delay(500);
  while(1){
    Serial.printf("Stop\n");
    wheels.stop();
  }

  

}

void setup() {
  Serial.begin(115200);
  

  IMU.initial();
  initial_motor();
  
  delay(1000);

  
}

unsigned long time_plot;
void plot(){

  if(Serial.available()) {
    input_string = Serial.readString();
    input_speed = input_string.toInt();

  }

  Target_1 = input_speed;
  Target_2 = input_speed;
  Target_3 = input_speed;

  // Drive_Motor1(input_speed);

  if(millis() - time_plot > 10){
    time_plot = millis();
    Serial.print("Target :");
    Serial.print(abs(Target_1));
    Serial.print("\t");
    Serial.print("Speed 1:");
    Serial.print(velocity_1);
    Serial.print("\t");

    // Serial.print("PID : ");
    // Serial.print(control_signal_1);
    // Serial.print("\t");

    // Serial.print("Enc : ");
    // Serial.print(enc_cnt[0]);

    // Serial.print("Speed 2:");
    // Serial.print(velocity_2);
    // Serial.print("\t");

    // Serial.print("Speed 3:");
    // Serial.print(velocity_3);
    Serial.println();    

  }
}

void check_obs(void){
    timer_obs = millis();
  while(1){
    if((control.control_obs_loop(0,90,100) > 50) && (millis()-timer_obs > 2500)){
      break;
    }

  }
  while(1){
    Serial.printf("Stop\n");
    wheels.stop();
  }

}


void loop() {
  

  plan2();

  // control.control_slope_loop(0,0,0);
  // IMU.call_bno();
  // navigation.head_navigate(90);
  // while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();

  // }
  // Serial.printf("BUF_L : %d BUF_R :%d\n",analogRead(BUF_L),analogRead(BUF_R));
  // control.control_obs_loop(0,0,0);
  // plan();
  // control.get_distance_vector();
  // delay(500);
  //   while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();

  // }

  // wheels.cmd_motor(50,0,0);
  // wheels.move(100,90,0);
  // Drive_Motor1(100);

  
  // Serial.printf("A: %d B: %d\n",digitalRead(28),digitalRead(29));

  // control.reset_time_slope();
  // control.control_slope_loop(0,90,60);


  // while(1){
  //   Serial.printf("Stop");
  //   wheels.stop();
  // }

  // plan();
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



  // 3062




  // -3.31 - > p
  // - 1.625 -> 
  // wheels.move(100,120,0);
  // control.cmd_motor(-50,-50,-50);
  // Serial.printf("ENC1: %d , ENC2: %d , ENC3: %d\n",enc_cnt[0],enc_cnt[1],enc_cnt[2]);


}