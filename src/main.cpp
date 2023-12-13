#include <Arduino.h>
#include <base.h>
#include <arm.h>

#define GREEN_SW 33
#define RETRY_SW 34
#define RED_SW 35
#define WHITE_SW 36

// OUT
#define LEDB 22
#define LEDG 23

unsigned long timer_obs;
unsigned long time_slope_reset;

float yaw_drop;

IMU_t IMU;
Wheels wheels;
Control control;
Navigation navigation;
Arm arm;
IntervalTimer obs_ISR;

char reactor = '0';

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
  time_slope_reset = millis();
  while(1){
    if(control.control_slope_loop(90,90,60,time_slope_reset)){
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
      delay(50);
      break;

    }
  }
}

void set_origin(int target_angle){
  while(1){
    if(analogRead(BUF_L) < 300 && analogRead(BUF_R) < 200){
      step_back(target_angle);
      delay(50);
      break;

    }
    else{
      control.control_heading(target_angle,90,140);
    }

  }
}

void set_origin_bside(int target_angle){
  while(1){
    if(analogRead(BUF_L) < 300 && analogRead(BUF_R) < 200){
      break;

    }
    else{
      control.control_heading(target_angle,90,130);
    }

  }
}

unsigned long time_ir;
void set_origin_side(void){
    while(1){
    if(analogRead(BUF_L) < 300 || analogRead(BUF_R) < 200){
      control.control_heading(270,0,110);

    }
    else{
      delay(70);
      break;
    }
  }
}

int alp_stand = 330;
unsigned long time_stand;

void set_stand(void){
  time_stand = millis();
  while(1){
  control.control_heading(90,alp_stand,60);
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
    while(millis() - time_stand < 200){
      IMU.call_bno();
      yaw_drop = IMU.yaw;
      Serial.printf("stand :%f\n",yaw_drop);
    }
    break;
    
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

void drop_square(void){
  unsigned long time_set;
  navigation.via_navigate(0.0,0.0,0,0,true);
  navigation.p2p_navigate(0.105,0.00,yaw_drop);
  while(1){
    control.control_heading(yaw_drop,270,40);
    if(!digitalRead(BUF_LIM_FL) && !digitalRead(BUF_LIM_FR)){
      if(millis() - time_set > 200){
        break;
      }
      
    }
    else{
      time_set = millis();
    }
  }
  arm.lift_down();
  wheels.stop();
  delay(500);
  
  arm.drop();
  wheels.stop();
  // while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();
  // }

  // navigation.p2p_navigate(0.0,0.058,yaw_drop,true);
  // find_hole();
  
  delay(200);
  navigation.via_navigate(0.0,0.0,0,0,true);
  navigation.p2p_navigate(0.0,0.058,yaw_drop,true);
  navigation.p2p_navigate(0.0,-0.01,yaw_drop,true);
  navigation.p2p_navigate(0.035,0.0,yaw_drop,true);
  navigation.p2p_navigate(-0.035,0.0,yaw_drop,true);
  navigation.p2p_navigate(0.0,0.0,yaw_drop,true);

  
  wheels.stop();
  delay(100);
  arm.ajar();
  
  navigation.via_navigate(0.0,0.1,yaw_drop,40,true);
  arm.lift_up();

}
void move_rot(int theta,float *x ,float *y, float curx,float cury){
  *x = curx*cos(theta*DEG_TO_RAD) - cury*sin(theta*DEG_TO_RAD);
  *y = curx*sin(theta*DEG_TO_RAD) + cury*cos(theta*DEG_TO_RAD);
  
}

void drop_hexagon(){
  navigation.via_navigate(0.0,0.0,0,0,true);
  navigation.p2p_navigate(0.19,0.00,yaw_drop);

  arm.lift_down();
  wheels.stop();
  delay(500);


  navigation.via_navigate(0.0,0.045,yaw_drop,40,true);

  float pos_x = 0.03f;
  float pos_y = 0.00f;
  navigation.p2p_navigate(pos_y,pos_x,0);
  // while(1){
  //   move_rot(45,&pos_x,&pos_y,pos_x,pos_y);
  //   navigation.p2p_navigate(pos_y,pos_x,yaw_drop);
  
  // }

  navigation.via_navigate(0.0,0.0,0,0,true);
  navigation.p2p_navigate(-0.04,-0.02,yaw_drop,true);
  navigation.p2p_navigate(0.035,-0.02,yaw_drop,true);
  navigation.p2p_navigate(0.0,0.035,yaw_drop,true);
  navigation.p2p_navigate(-0.052,-0.02,yaw_drop,true);

  wheels.stop();
  delay(100);
  arm.ajar();
  
  navigation.via_navigate(0.0,0.1,yaw_drop,40,true);
  arm.lift_up();

}

float xp[100];
float yp[100];

float first_y = 0.0;
float first_x = 0.06;


float dropx;
float dropy;


void rot(int theta){
  xp[0] = first_x;
  yp[0] = first_y;

  for(int i = 1 ; i < 36; i++){
    xp[i] = xp[i-1]*cos(theta*DEG_TO_RAD) - yp[i-1]*sin(theta*DEG_TO_RAD);
    yp[i] = xp[i-1]*sin(theta*DEG_TO_RAD) + yp[i-1]*cos(theta*DEG_TO_RAD);
    navigation.p2p_navigate(yp[i],xp[i],0);
    Serial.printf("x = %f y = %f\n",xp[i],yp[i]);
  }
  // Serial.printf("x = %f y = %f\n",xp[3],yp[3]);
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







void holonomic_plan(char select){
  // navigation.via_navigate(0.0,0.0,0,0,true);
  navigation.head_navigate(0);
  navigation.via_navigate(0.10,0.0,0,140);
  navigation.via_navigate(0.37,0.0,0,220);
  navigation.via_navigate(0.47,0.0,0,170);
  wheels.stop();
  delay(100);

  arm.set();
  delay(150);
  arm.grip();
  navigation.via_navigate(0.47,-0.10,0,140);
  navigation.via_navigate(0.47,-0.40,90,140);
  navigation.via_navigate(0.47,-0.52,180,140);
  navigation.head_navigate(180);
  wheels.stop();
  delay(200);

  timer_obs = millis();
  while(1){
    if((control.control_obs_loop(180,90,120) > 50) && (millis()-timer_obs > 2500)){
      break;
    }

  
  }
  wheels.stop();
  delay(200);
  set_origin(180);
  navigation.head_navigate(180);
  delay(200);
  
  navigation.via_navigate(-0.01,-0.13,180,125,true);
  navigation.via_navigate(-0.01,-0.53,180,190,true);
  navigation.via_navigate(-0.01,-0.73,180,120);
  delay(80);

  // control.time_slope = millis();
  wheels.stop();
  delay(80);
  time_slope_reset = millis();
  while(1){
    if(control.control_side_slope_loop(180,0,100,time_slope_reset)){
      break;
    }

  }

  //   while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();

  // }
  navigation.via_navigate(0.04,0.0,180,90,true);
  wheels.stop();
  delay(100);
  // navigation.head_navigate(270);
  navigation.via_navigate(0.0,-0.2,240,90,true);
  set_origin_bside(270);
  set_origin_side();
  wheels.stop();
  delay(100);

  set_origin(270);

  navigation.via_navigate(-0.162,0.0,265,120,true);
  navigation.via_navigate(-0.47,-0.1,260,140);
  navigation.via_navigate(-0.5,-0.8,260,130);
  navigation.via_navigate(0.2,-0.2,245,160,true);
  navigation.via_navigate(0.25,0.0,225,220,true);
  navigation.via_navigate(0.15,0.1,215,220,true);
  navigation.via_navigate(0.12,0.15,205,220,true);
  navigation.via_navigate(0.0,0.0,160,220,true);
  navigation.via_navigate(0.0,0.44,130,160,true);
  navigation.head_navigate(100);

  // Road to drop
  navigation.via_navigate(-0.14,0.0,90,70,true);

  set_stand();
  wheels.stop();
  delay(200);
  switch (reactor)
  {
  case '1':
    drop_circle();
    break;
  
  case '2':
    drop_square();
    break;

  case '3':
    drop_hexagon();
    break;
  }
  navigation.via_navigate(0.0,0.0,0,0,true);
  control.pass_slope = false;
  
}




void retry_run(){
  wheels.stop();
  while(arm.read_tof() > 40){
    Serial.printf("Wait Robojames\n");
  }
  digitalWrite(LEDB,HIGH);
  delay(200);
  arm.grip();
  wheels.stop();
  delay(500);

  while(1){
    if(!digitalRead(RED_SW)){
      reactor = '1';
      break;
    }
    else if(!digitalRead(WHITE_SW)){
      reactor = '2';
      break;
    }
    else if(!digitalRead(GREEN_SW)){
      reactor = '3';
      break;
    }
  }
  digitalWrite(LEDG,HIGH);


  delay(500);
  arm.lift_up();


  navigation.head_navigate(270);
  arm.set();
  delay(150);
  arm.grip();
  navigation.via_navigate(0.0,0.0,270,110);
  wheels.stop();
  set_origin(270);

  navigation.via_navigate(-0.162,0.0,265,110,true);
  navigation.via_navigate(-0.47,-0.1,260,130);
  navigation.via_navigate(-0.5,-0.8,260,130);
  navigation.via_navigate(0.2,-0.2,245,130,true);
  navigation.via_navigate(0.25,0.0,225,130,true);
  navigation.via_navigate(0.15,0.1,215,130,true);
  navigation.via_navigate(0.12,0.15,205,130,true);
  navigation.via_navigate(0.0,0.0,160,130,true);
  navigation.via_navigate(0.0,0.43,130,130,true);
  navigation.head_navigate(100);

  // Road to drop
  navigation.via_navigate(-0.14,0.0,90,70,true);

  set_stand();
  wheels.stop();
  delay(200);
  switch (reactor)
  {
  case '1':
    drop_circle();
    break;
  
  case '2':
    drop_square();
    break;

  case '3':
    drop_hexagon();
    break;
  }
  navigation.via_navigate(0.0,0.0,0,0,true);
  control.pass_slope = false;
}


void run_robot(void){
  wheels.stop();
  arm.lift_up();
  while(1){
    if(!digitalRead(RED_SW)){
      reactor = '1';
      break;
    }
    else if(!digitalRead(WHITE_SW)){
      reactor = '2';
      break;
    }
    else if(!digitalRead(GREEN_SW)){
      reactor = '3';
      break;
    }
  }
  digitalWrite(LEDB,HIGH);
  delay(500);
  navigation.head_navigate(45);

  delay(500);
  gripper.write(40);
  lift.write(170);

  wheels.stop();
  while(arm.read_tof() > 40){
    Serial.printf("Wait Robojames\n");
  }
  digitalWrite(LEDG,HIGH);
  delay(800);
  arm.grip();
  delay(1500);
  arm.lift_up();
  delay(1000);
  arm.lift_up();
  holonomic_plan(reactor);

}



void retry_run_start(void){
  wheels.stop();
  arm.lift_up();
  arm.grip();
  while(1){
    if(!digitalRead(RED_SW)){
      reactor = '1';
      break;
    }
    else if(!digitalRead(WHITE_SW)){
      reactor = '2';
      break;
    }
    else if(!digitalRead(GREEN_SW)){
      reactor = '3';
      break;
    }
  }
  digitalWrite(LEDB,HIGH);

  delay(500);
  wheels.stop();
  delay(1000);
  holonomic_plan(reactor);

}

void find_hole(){
  static float xpath = -0.04;
  static float ypath = 0.00;
  

  for(int i=0; i < 3 ; i++){
    // origin
    navigation.via_navigate(0.0,0.0,90,0,true);

    // D
    move_rot(180,&xpath,&ypath,xpath,ypath);
    navigation.p2p_navigate(ypath,xpath,90,true);

    // U
    move_rot(180,&xpath,&ypath,xpath,ypath);
    navigation.p2p_navigate(ypath,xpath,90,true);

    // origin
    navigation.p2p_navigate(0,0,90,true);
    
    // <
    move_rot(90,&xpath,&ypath,xpath,ypath);
    navigation.p2p_navigate(ypath,xpath,90,true);

    // >
    move_rot(180,&xpath,&ypath,xpath,ypath);
    navigation.p2p_navigate(ypath,xpath,90,true);

    // wheels.stop();
    // delay(100);
    // arm.ajar();
    
    // navigation.via_navigate(0.0,0.1,yaw_drop,40,true);
    // arm.lift_up();


  }


}

boolean retry = false;

void setup() {
  Serial.begin(115200);
  IMU.initial();
  initial_base();
  arm.initial();
  pinMode(GREEN_SW,INPUT);
  pinMode(RED_SW,INPUT);
  pinMode(WHITE_SW,INPUT);
  pinMode(RETRY_SW,INPUT);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDG,OUTPUT);

  pinMode(ARM_LIM,INPUT);

  digitalWrite(LEDB,LOW);
  digitalWrite(LEDG,LOW);
  
  if(!digitalRead(RETRY_SW)){
    retry = true;
  }


  // while(arm.read_tof() > 40){
  //   Serial.printf("Wait Robojames\n");
  // }
  // delay(800);
  // arm.grip();
  // delay(1500);
  // arm.lift_up();
  // delay(1000);
  // arm.lift_up();
}

void loop(){
  
  // Serial.printf("%d %d\n",analogRead(BUF_L),analogRead(BUF_R)); // Monitor

  // navigation.p2p_navigate(1.0,0.0,0,true);
  // while(1){
  //   Serial.printf("Stop\n");
  //   wheels.stop();
  // }
  if(retry){
    retry_run_start();
  }
  else{
    run_robot();
  }

}