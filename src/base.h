#include <Arduino.h>
#include <speed_control.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
unsigned long sample_time_imu;

IntervalTimer ramping_timer;


float vr = 100;
float alp = 90;

float _x = 0, _y = 0, w = 0;
float l = 0.089;



class IMU_t {

  public:
    float yaw, pitch, roll,acel_z,vel_z,pos_z,prev_acel_z;
    unsigned long time_pos =0 ;

    void displaySensorDetails(void) {
      sensor_t sensor;
      bno.getSensor(&sensor);
      Serial.println("------------------------------------");
      Serial.print  ("Sensor:       "); Serial.println(sensor.name);
      Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
      Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
      Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
      Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
      Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
      Serial.println("------------------------------------");
      Serial.println("");
      delay(500);
    }

    void displaySensorStatus(void) {
      /* Get the system status values (mostly for debugging purposes) */
      uint8_t system_status, self_test_results, system_error;
      system_status = self_test_results = system_error = 0;
      bno.getSystemStatus(&system_status, &self_test_results, &system_error);

      /* Display the results in the Serial Monitor */
      Serial.println("");
      Serial.print("System Status: 0x");
      Serial.println(system_status, HEX);
      Serial.print("Self Test:     0x");
      Serial.println(self_test_results, HEX);
      Serial.print("System Error:  0x");
      Serial.println(system_error, HEX);
      Serial.println("");
      delay(500);
    }

    void displayCalStatus(void) {
      /* Get the four calibration values (0..3) */
      /* Any sensor data reporting 0 should be ignored, */
      /* 3 means 'fully calibrated" */
      uint8_t system, gyro, accel, mag;
      system = gyro = accel = mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);

      /* The data should be ignored until the system calibration is > 0 */
      Serial.print("\t");
      if (!system)
      {
        Serial.print("! ");
      }

      /* Display the individual values */
      Serial.print("Sys:");
      Serial.print(system, DEC);
      Serial.print(" G:");
      Serial.print(gyro, DEC);
      Serial.print(" A:");
      Serial.print(accel, DEC);
      Serial.print(" M:");
      Serial.print(mag, DEC);
    }

    void initial(void) {
      /* Initialise the sensor */
      if (!bno.begin())
      {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
      }

      delay(1000);

      /* Display some basic information on this sensor */
      displaySensorDetails();

      /* Optional: Display current status */
      displaySensorStatus();

      bno.setExtCrystalUse(true);

    }

    void call_bno(void) {

      if (millis() - sample_time_imu >= BNO055_SAMPLERATE_DELAY_MS) {
        sample_time_imu = millis();

        sensors_event_t event;
        bno.getEvent(&event);


        yaw = event.orientation.x;
        pitch = event.orientation.y;
        roll = event.orientation.z;

        // acel_z = event.acceleration.z;
        // vel_z = (acel_z)*0.1+0.153;
        // pos_z = vel_z*0.1;
        
        // Serial.printf("POS_Z : %f\n",vel_z);
        // prev_acel_z = acel_z;
        



               Serial.print("Yaw: ");
               Serial.print(yaw);
               Serial.print("\tPitch: ");
               Serial.print(pitch);
               Serial.print("\tRoll: ");
               Serial.print(roll);
               Serial.println();


      }
    }


};


class Wheels{
    float rpm[3];
    float* wheel_speed; 
    float ramp;


    public:
        void cmd_motor(int speed1 , int speed2, int speed3){
            Target_1 = speed1;
            Target_2 = speed2;
            Target_3 = speed3;
        }
        void stop(void){
            Target_1 = 0;
            Target_2 = 0;
            Target_3 = 0;

        }

        float* cal_wheels_rpm(float vr, float alp, float omega) {
            _x = vr * cos(alp * DEG_TO_RAD);
            _y = vr * sin(alp * DEG_TO_RAD);
            w = omega;

            // rpm[0] = -_x + l * w;
            // rpm[1] = _x * cos(300 * DEG_TO_RAD) + _y * sin(300 * DEG_TO_RAD) + l * w;
            // rpm[2] = _x * cos(60 * DEG_TO_RAD) + _y * sin(60 * DEG_TO_RAD) + l * w;

            rpm[0] = vr*cos((180-alp)*DEG_TO_RAD)+l*w;
            rpm[1] = vr*cos((300-alp)*DEG_TO_RAD)+l*w;
            rpm[2] = vr*cos((60-alp)*DEG_TO_RAD)+l*w;

            // rpm[0] = -vr*cos(alp*DEG_TO_RAD) + l*w;
            // rpm[1] = vr*(cos(300 * DEG_TO_RAD)*cos(alp*DEG_TO_RAD) +sin(300*DEG_TO_RAD)*sin(alp*DEG_TO_RAD)) + l*w;
            // rpm[2] = vr*(cos(60*DEG_TO_RAD)*cos(alp*DEG_TO_RAD) + sin(60*DEG_TO_RAD)*sin(alp*DEG_TO_RAD)) + l*w;

            // Serial.printf("%f , %f , %f \n",rpm[0],rpm[1],rpm[2]);

            return rpm;

        }
        void move(int speed ,int alpha ,int W){
            wheel_speed = cal_wheels_rpm(speed,alpha,W);
            cmd_motor(*(wheel_speed),*(wheel_speed+1)  ,*(wheel_speed+2));
            // cmd_motor(0,-86,86);
            //  multiple_write(1 , 0 , 2 , -100 , 3 , 100);

            // Serial.printf("speed = %f %f %f\n",*(rpm)* 1,*(rpm+1)* 1,*(rpm+2)* 1);
        }
        void move_ramp(int speed ,int alpha ,int W){
          ramp += 0.015;
          if(ramp > speed){
            ramp = speed;
          }
          wheel_speed = cal_wheels_rpm(ramp,alpha,W);
          cmd_motor(*(wheel_speed),*(wheel_speed+1)  ,*(wheel_speed+2));

          Serial.printf("Ramping : %f\n",ramp);

        }

        void reset_ramp(void){
            ramp = 0.0f;
        }

        
};

class Control : public Wheels , public IMU_t{

    
    unsigned long imu_current_time,time,imu_last_time,imu_dt_time;

    float kp_imu = 16.0;
    float ki_imu = 0.0001; // 35.5
    float kd_imu = 0.0;

    const float consy = 3167.28f;
    const float consx = 3140.66f;
    
    float p2p_kp = 150.0 ,p2p_ki = 0.001 ,p2p_kd = 0.05;
    // float p2p_kp = 100.0 ,p2p_ki = 0.0 ,p2p_kd = 0.00;
    
    unsigned long dt;
    
    int cnt_obs;
    float diff_pitch,diff_roll;
    unsigned long time_obs;
    float prev_pitch , prev_roll;

    
    bool pass_slope = false;

    
    
    public:
        int cnt_slope = 0;

        float imu_error , imu_error_sum,imu_error_diff,imu_prev_error,imu_control=0.0f;
        float* distance;
        uint8_t p2p_cnt;
        uint8_t head_cnt;
        float p2p_sum_error ,p2p_prev_disp;
        float pvector[2] = {0.0f,0.0f};
        float dvector[2];
        float px,py;
        float error_heading;

        unsigned long time_slope;
        



        float control_heading(int target_angle ,int alp , float vr , bool ramp = false){
            imu_current_time = micros();
            call_bno();      
            imu_error = target_angle - yaw;
            
    
            if (imu_error < - 180) {
                imu_error += 360;
            }
            if (imu_error > 180) {
                imu_error -= 360;
            }
            imu_error_sum += imu_error;
            if(fabs(imu_error) < 1.0f) imu_error_sum = 0;
            imu_error_sum = constrain(imu_error_sum,-45000,45000);
            imu_error_diff = imu_error - imu_prev_error;
            // imu_control = kp * imu_error + ki * imu_error_sum * imu_dt_time + kd * imu_error_diff / imu_dt_time;
            imu_control = kp_imu*imu_error + ki_imu * imu_error_sum  + kd_imu * imu_error_diff;

            imu_control = constrain(imu_control, -600, 600);
            if(ramp){
              move_ramp(vr,alp,imu_control*-1);
            }
            else{
            move(vr,alp,imu_control*-1);
            }
            // Serial.printf("yaw = %f error= %f \t control= %f  sum_error= %f termI= %f\n",yaw,imu_error,imu_control,imu_error_sum,ki_imu*imu_error_sum);

            imu_prev_error = imu_error;

            imu_dt_time = ((imu_current_time - imu_last_time) / 1.0e6 ) + 0.001f;

            imu_last_time = imu_current_time;
            
            return imu_error;
        }

        int control_obs_loop(int target_angle ,int alp , float vr){
          call_bno(); 
          if(millis() - time_obs > 10){
            time_obs = millis();
            cnt_obs ++;
            // Serial.printf("cnt : %d\n",cnt_obs);
            // if(cnt_obs > 60){
            //   break;
            // }

            diff_pitch = fabs(pitch - prev_pitch);
            diff_roll = fabs(roll - prev_roll);
            prev_pitch = pitch;
            prev_roll = roll;

            if(diff_pitch > 0.7 || diff_roll > 0.6|| fabs(-0.25 -pitch) > 3.0 || fabs(-1.0 - roll) > 3.0){
              Serial.printf("diff pitch : %f diff roll :%f pitch: %f cnt :%f\n ",pitch - prev_pitch,roll - prev_roll , pitch,cnt_obs);
              cnt_obs = 0;
            }

            imu_current_time = micros();
            imu_error = target_angle - yaw;
            
    
            if (imu_error < - 180) {
                imu_error += 360;
            }
            if (imu_error > 180) {
                imu_error -= 360;
            }
            imu_error_sum += imu_error;
            if(fabs(imu_error) < 1.0f) imu_error_sum = 0;
            imu_error_sum = constrain(imu_error_sum,-60000,60000);
            imu_error_diff = imu_error - imu_prev_error;
            // imu_control = kp * imu_error + ki * imu_error_sum * imu_dt_time + kd * imu_error_diff / imu_dt_time;
            imu_control = kp_imu*imu_error + ki_imu * imu_error_sum  + kd_imu * imu_error_diff;

            imu_control = constrain(imu_control, -500, 500);
        
            move(vr,alp,imu_control*-1);
            
            // Serial.printf("yaw = %f error= %f \t control= %f  sum_error= %f termI= %f\n",yaw,imu_error,imu_control,imu_error_sum,ki_imu*imu_error_sum);

            imu_prev_error = imu_error;

            imu_dt_time = ((imu_current_time - imu_last_time) / 1.0e6 ) + 0.001f;

            imu_last_time = imu_current_time;
        }
        return cnt_obs;
          
        }
        void reset_time_slope(void){
          time_slope = 0;

        }

        bool control_slope_loop(int target_angle ,int alp , float vr){
          call_bno(); 
          if(fabs(0.06-pitch) <1.10 && fabs(-1.63-roll) < 3.10){
            Serial.printf("Normal cnt: %d :: %f\n",cnt_slope,-3.85-pitch);
            if(cnt_slope == 2 || cnt_slope == 3 || cnt_slope == 4){
              pass_slope = true;
            }

          }
          else if(0.06-pitch < -1.10 && fabs(-1.63-roll) < 3.10){
            Serial.printf("Take off %f\n",-3.85-pitch);
            cnt_slope = 1;

          }
          else if(0.06 - pitch > 1.10 && fabs(-1.63-roll) < 3.10){
            Serial.printf("Landing %f\n",-3.85-pitch);
            cnt_slope = 2;
          }
          else if(fabs(-1.63-roll) > 3.10){
            Serial.printf("Anomal %f\n",-1.63-roll); 
            cnt_slope = 2;
            // if(-1.63-roll < 0){
            //   cnt_slope = 3;
            // }
            // else if(-1.63-roll > 0){
            //   cnt_slope = 4;
            // }
          }

          imu_current_time = micros();
          imu_error = target_angle - yaw;
            
    
          if (imu_error < - 180) {
              imu_error += 360;
          }
          if (imu_error > 180) {
              imu_error -= 360;
          }
          imu_error_sum += imu_error;
          if(fabs(imu_error) < 1.0f) imu_error_sum = 0;
          imu_error_sum = constrain(imu_error_sum,-60000,60000);
          imu_error_diff = imu_error - imu_prev_error;
            // imu_control = kp * imu_error + ki * imu_error_sum * imu_dt_time + kd * imu_error_diff / imu_dt_time;
          imu_control = kp_imu*imu_error + ki_imu * imu_error_sum  + kd_imu * imu_error_diff;

          imu_control = constrain(imu_control, -500, 500);
        
          move(vr,alp,imu_control*-1);
            
            // Serial.printf("yaw = %f error= %f \t control= %f  sum_error= %f termI= %f\n",yaw,imu_error,imu_control,imu_error_sum,ki_imu*imu_error_sum);

          imu_prev_error = imu_error;

          imu_dt_time = ((imu_current_time - imu_last_time) / 1.0e6 ) + 0.001f;

          imu_last_time = imu_current_time;
          // else if(fabs(-3.85-pitch) >5.00 || fabs(-2.25-roll)> 4.80){
          //   Serial.printf("Anormal \n");
          //   cnt_slope = 1;

          // }
          return pass_slope;

        }

        float* get_distance_vector(void){

            call_bno();
            // Serial.printf("%f\n",yawValue);
            // dvector[0] = (kENC1*(enc_cnt[0])*sin(120*DEG_TO_RAD) + kENC2*(enc_cnt[1])*sin(240*DEG_TO_RAD)) * 0.666666667; //y 
            // dvector[1] = (kENC1*(enc_cnt[0])*cos(120*DEG_TO_RAD) + kENC2 * (enc_cnt[1])*cos(240*DEG_TO_RAD) * kENC2 + enc_cnt[2] *kENC3)*0.78125; // x
            // Serial.printf("%d , %d , %d , %f , %f\n",enc_cnt[0],enc_cnt[1],enc_cnt[2],pvector[0],pvector[1]);// Serial.println(pvector[0]);
    
            float raw_y = (enc_cnt[1]*sin(300*DEG_TO_RAD)) + (enc_cnt[2]*sin(60*DEG_TO_RAD));
            float raw_x = (enc_cnt[0]*cos(180*DEG_TO_RAD)) + (enc_cnt[1]*cos(300*DEG_TO_RAD)) +(enc_cnt[2]*cos(60*DEG_TO_RAD));
            
            float y = raw_y/consy;
            float x = raw_x/consx;
            
            dvector[0] = pvector[0] + (x-px)*sin((360 - yaw)*DEG_TO_RAD) +  (y-py)*cos((360 - yaw)*DEG_TO_RAD); //y 
            dvector[1] = pvector[1] + (x-px)*cos((360 - yaw)*DEG_TO_RAD) - (y-py)*sin((360 - yaw)*DEG_TO_RAD);//x

            // Serial.printf("%d , %d , %d x =%f , y = %f , cx = %f , cy = %f yaw = %f\n",enc_cnt[0],enc_cnt[1],enc_cnt[2],x ,y,dvector[1],dvector[0],yawValue);
            px = x;
            py = y;
            pvector[0] = dvector[0];
            pvector[1] = dvector[1];
            Serial.printf("rawY = %f rawX = %f y = %f x = %f yaw = %f\n",raw_y,raw_x,y,x,yaw);

            // Serial.printf("Y = %f X = %f Transf_Y = %f Transf_x = %f yaw = %f enc1 :%f\n",y,x,dvector[0],dvector[1],yaw,enc_cnt[0]);
            return dvector;
            // Serial.printf("%d , %d , %d , %f , %f\n",enc_cnt[0],enc_cnt[1],enc_cnt[2],pvector[0],pvector[1]);// Serial.println(pvector[0]);
        
        }


        float p2p(float yg ,float xg ,int angle){
            distance = get_distance_vector();
            float x = *(distance+1);
            float y = *distance;

            float disp_error = sqrt((xg-x)*(xg-x) + (yg-y)*(yg-y));
            float alp = fmod((360 + (atan2(yg-y, xg-x) * RAD_TO_DEG)), 360);

            unsigned long current_time = micros();
            p2p_sum_error += disp_error;

            float control_displecement  =  p2p_kp*disp_error + p2p_ki*p2p_sum_error + p2p_kd*(disp_error - p2p_prev_disp)/dt;
            
            error_heading = control_heading(angle,fmod(alp+yaw,360),control_displecement);
            if(disp_error < 0.005)p2p_sum_error=0;
            // Serial.printf("y = %f ,x = %f\n",dvector[0],dvector[1]);
            p2p_prev_disp = disp_error;
            dt = current_time;

            // Serial.printf("error_yaw = %f error = %f sum_heading = %f control = %f sum_error = %f p2p_cnt = %d\n",error_heading,disp_error,imu_error_sum,control_displecement,p2p_sum_error,p2p_cnt);

            // float detlas = sqrt(pow(pos.x-posx_prev,2)+pow(pos.y-posy_prev,2));
            // vel = deltas/0.1;
            // posx_prev = pos.x;
            // posy_prev = pos.y;
            return disp_error;

        
    }
    void reset_odom(){
      dvector[0] = 0.0;
      dvector[1] = 0.0;
      pvector[0] = 0.0;
      pvector[1] = 0.0;
      enc_cnt[0] = 0.0;
      enc_cnt[1] = 0.0;
      enc_cnt[2] = 0.0;
      px = 0.0;
      py = 0.0;
    }

      

};

class Navigation : public Control{
  
  public:
    void via_navigate(float yg , float xg ,int angle ,float speed,bool reset = false,bool ramp=false){
      if(reset){
        *distance = 0.0;
        *(distance+1) = 0.0;
        reset_odom();

      }
      while(1){
        distance = get_distance_vector();
        float x = *(distance+1);
        float y = *distance;
        float disp_error = sqrt((xg-x)*(xg-x) + (yg-y)*(yg-y));
        float alp = fmod((360 + (atan2(yg-y, xg-x) * RAD_TO_DEG)), 360);
        error_heading = control_heading(angle,fmod(alp+yaw,360),speed,ramp);
        if(disp_error < 0.025){
          break;
        }
        // Serial.printf("y = %f x = %f Alpha = %f error_yaw = %f error = %f\n",y,x,alp,error_heading,disp_error);
      }
        
  }
  void p2p_navigate(float yg , float xg , int angle){
    p2p_cnt = 0;
    p2p_sum_error = 0;
    imu_error_sum =0;

    
    while(1){
      if(p2p(yg,xg,angle) < 0.05){
        if(++p2p_cnt >= 200)break;
      }
    }
  }

  void head_navigate(float target_angle){
    head_cnt = 0;
    imu_error_sum =0;
    while(1){
      Serial.printf("In heading loop : %f\n",yaw);
      if(fabs(control_heading(target_angle,0,0)) < 1.05){
          if(++head_cnt >= 200)break;
      }

    }

  }

};