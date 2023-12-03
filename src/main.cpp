#include <Arduino.h>
#include <base.h>
#include <arm.h>

unsigned long timer_obs;

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;
Arm arm;

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
      control.control_heading(target_angle,90,100);
    }

  }
}

void set_origin_bside(int target_angle){
  while(1){
    if(analogRead(BUF_L) < 300 && analogRead(BUF_R) < 200){
      break;

    }
    else{
      control.control_heading(target_angle,90,100);
    }

  }
}

unsigned long time_ir;
void set_origin_side(void){
    while(1){
    if(analogRead(BUF_L) < 300 || analogRead(BUF_R) < 200){
      control.control_heading(270,0,100);

    }
    else{
      delay(70);
      break;
    }
  }
}

int alp_stand = 330;
unsigned long time_stand;

void set_stand_circle(void){
  time_stand = millis();
  while(1){
  control.control_heading(90,alp_stand,50);
  if(!digitalRead(BUF_LIM_SB) && !digitalRead(BUF_LIM_SF)){
    alp_stand = 290;
  }
  else if(!digitalRead(BUF_LIM_FL) && !digitalRead(BUF_LIM_FR)){
    alp_stand = 350;
  }
  else{
    alp_stand = 330;
  }

  if(!digitalRead(BUF_LIM_FL) && !digitalRead(BUF_LIM_FR) && !digitalRead(BUF_LIM_SB) && !digitalRead(BUF_LIM_SF)){
    if(millis() - time_stand > 200){
      break;
    }
    
  }
  else{
    time_stand = millis();
  }
  }
}

void drop_circle(void){
  arm.lift_down();
  wheels.stop();
  delay(500);
  navigation.via_navigate(0.0,0.045,80,40,true);
  arm.drop();
  wheels.stop();
  arm.ajar();
  delay(200);
  navigation.via_navigate(0.0,0.1,85,40,true);
  arm.lift_up();


}

void plan2(void){
  navigation.via_navigate(0.51,0.0,0,120);
  wheels.stop();
  delay(200);

  arm.drop();
  delay(200);
  arm.grip();
  navigation.via_navigate(0.5,-0.52,0,120);
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
  navigation.via_navigate(0.03,0.0,270,90,true);

  set_origin(270);
  navigation.head_navigate(0);
  set_origin(0);
  navigation.head_navigate(270);
  set_origin(270);

  navigation.via_navigate(-0.140,0.0,265,120,true);
  navigation.via_navigate(-0.45,-0.1,265,120);
  navigation.via_navigate(-0.5,-0.8,265,120);
  navigation.via_navigate(0.2,-0.2,265,120,true);
  navigation.via_navigate(0.25,0.0,265,120,true);
  navigation.via_navigate(0.15,0.15,265,120,true);
  navigation.via_navigate(0.15,0.15,265,120,true);
  navigation.via_navigate(0.0,0.0,265,120,true);
  navigation.via_navigate(0.0,0.4,265,80,true);
  navigation.head_navigate(90);

  // Road to drop
  navigation.via_navigate(-0.15,0.0,90,70,true);

  set_stand_circle();
  drop_circle();


  while(1){
    Serial.printf("Stop\n");
    wheels.stop();
  }
}


void plan3(){
  navigation.via_navigate(0.51,0.0,0,100);
  wheels.stop();
  delay(200);

  navigation.via_navigate(0.5,-0.52,0,100);
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
  navigation.via_navigate(-0.02,-0.5,270,90,true);
  delay(200);
  while(1){
    if(control.control_slope_loop(270,90,60)){
      break;
    }

  }

  navigation.via_navigate(0.03,0.0,270,90,true);
  
  set_origin(270);
  navigation.head_navigate(0);
  set_origin(0);
  navigation.head_navigate(270);
  set_origin(270);

  navigation.via_navigate(-0.2,0.027,270,120,true);
  navigation.via_navigate(0.0,0.0,270,120,true);

  float xp[] = {0.0, -0.03, -0.05, -0.08, -0.1, -0.12, -0.15, -0.17, -0.2, -0.22, -0.25, -0.2};
  float yp[] = {-0.0, -0.12, -0.17, -0.2, -0.22, -0.24, -0.26, -0.27, -0.28, -0.29, -0.3, -0.3};
  for(int i = 0 ; i < 12 ; i++){
    navigation.via_navigate(yp[i],xp[i],270,100);
  }
  navigation.via_navigate(0.0,-0.55,270,120,true);

  // float xp[] = {-0.0, -0.13, -0.18, -0.22, -0.24, -0.27, -0.29, -0.3, -0.32, -0.33, -0.34, -0.34, -0.35, -0.35, -0.35, -0.35, -0.35, -0.34, -0.34, -0.33, -0.32, -0.3, -0.29, -0.27, -0.24, -0.22, -0.18, -0.13};
  // float yp[] = {0.0, 0.03, 0.05, 0.08, 0.1, 0.12, 0.15, 0.17, 0.2, 0.22, 0.25, 0.27, 0.3, 0.33, 0.35, 0.38, 0.4, 0.43, 0.45, 0.48, 0.5, 0.53, 0.55, 0.58, 0.6, 0.63, 0.65, 0.68};
  // navigation.via_navigate(0.0,0.0,270,120,true);
  // for(int i = 0 ; i < 28; i++){
  //   navigation.via_navigate(yp[i],xp[i],270,100);
  // }

  while(1){
    Serial.printf("Stop\n");
    wheels.stop();
  }
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


void holonomic_plan(void){
  navigation.via_navigate(0.47,0.0,0,120);
  wheels.stop();
  delay(200);

  arm.drop();
  delay(200);
  arm.grip();
  navigation.via_navigate(0.47,-0.10,0,120);
  navigation.via_navigate(0.47,-0.40,90,120);
  navigation.via_navigate(0.47,-0.52,180,120);
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
  navigation.via_navigate(-0.025,-0.5,270,100,true);
  delay(200);
  while(1){
    if(control.control_slope_loop(270,90,60)){
      break;
    }

  }
  navigation.via_navigate(0.03,0.0,270,90,true);
  wheels.stop();
  delay(100);
  set_origin_bside(270);
  set_origin_side();
  wheels.stop();
  delay(100);
  set_origin(270);


  navigation.via_navigate(-0.150,0.0,265,120,true);
  navigation.via_navigate(-0.45,-0.1,260,120);
  navigation.via_navigate(-0.5,-0.8,260,120);
  navigation.via_navigate(0.2,-0.2,245,120,true);
  navigation.via_navigate(0.25,0.0,225,120,true);
  navigation.via_navigate(0.15,0.15,215,120,true);
  navigation.via_navigate(0.15,0.15,205,120,true);
  navigation.via_navigate(0.0,0.0,160,120,true);
  navigation.via_navigate(0.0,0.4,130,80,true);
  navigation.head_navigate(100);

  // Road to drop
  navigation.via_navigate(-0.15,0.0,90,70,true);

  set_stand_circle();
  drop_circle();


  while(1){
    Serial.printf("Stop\n");
    wheels.stop();
  }


}





void setup() {
  Serial.begin(115200);
  IMU.initial();
  initial_base();
  arm.initial();


  while(arm.read_tof() > 40){
    Serial.printf("Wait Robojames\n");
  }
  delay(800);
  arm.grip();
  delay(1500);
  arm.lift_up();
  delay(1000);
  // arm.lift_up();


  
}

void loop() {
  // set_stand_circle();


  // set_stand();
  // while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();
  // }
  // Serial.printf("%d %d %d %d\n",digitalRead(BUF_LIM_SB),digitalRead(BUF_LIM_SF),digitalRead(BUF_LIM_FL),digitalRead(BUF_LIM_FR));
  // navigation.via_navigate(1.0,0.0,0,60);
  // for(int i = 0 ; i < 28; i++){
  //   navigation.via_navigate(yp[i],xp[i],0,100);

  // }

  // while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();
  // }

  // control.control_slope_loop(0,0,0);
  // plan2();
  holonomic_plan();
// set_origin_bside(0);
// set_origin_side();

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

  // wheels.cmd_motor(20,20,20);
  // wheels.move(100,90,0);

  // Drive_Motor1(-500);
  // Drive_Motor2(-500);
  // Drive_Motor3(-500);
  
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
    // control.control_heading(0,90,100,true);

  // }



  // 3062




  // -3.31 - > p
  // - 1.625 -> 
  // wheels.move(100,120,0);
  // Serial.printf("ENC1: %d , ENC2: %d , ENC3: %d\n",enc_cnt[0],enc_cnt[1],enc_cnt[2]);


}