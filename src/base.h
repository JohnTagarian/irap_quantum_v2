#include <Arduino.h>
#include <speed_control.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BNO055_SAMPLERATE_DELAY_MS (99)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
unsigned long sample_time_imu;


float vr = 100;
float alp = 90;

float _x = 0, _y = 0, w = 0;
float l = 0.089;

class IMU_t {

  public:
    float yaw, pitch, roll;

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



            //    Serial.print("Yaw: ");
            //    Serial.print(yaw);
            //    Serial.print("\tPitch: ");
            //    Serial.print(pitch);
            //    Serial.print("\tRoll: ");
            //    Serial.print(roll);
            //    Serial.println();


      }
    }


};

class Wheels{
    float rpm[3];
    float* wheel_speed; 
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

        
};

class Control : public Wheels , public IMU_t{

    float imu_error , imu_error_sum,imu_error_diff,imu_prev_error,imu_control=0.0f;
    unsigned long imu_current_time,time,imu_last_time,imu_dt_time;

    float kp_imu = 5.0;
    float ki_imu = 0.000; // 35.5
    float kd_imu = 0.0;

    public:
        float control_heading(int target_angle ,int alp , float vr){
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
            imu_error_sum = constrain(imu_error_sum,-2000,2000);
            imu_error_diff = imu_error - imu_prev_error;
            // imu_control = kp * imu_error + ki * imu_error_sum * imu_dt_time + kd * imu_error_diff / imu_dt_time;
            imu_control = kp_imu*imu_error + ki_imu * imu_error_sum  + kd_imu * imu_error_diff;

            imu_control = constrain(imu_control, -500, 500);
            move(vr,alp,imu_control*-1);
            Serial.printf("yaw = %f error= %f \t control= %f  sum_error= %f termI= %f\n",yaw,imu_error,imu_control,imu_error_sum,ki_imu*imu_error_sum);

            imu_prev_error = imu_error;

            imu_dt_time = ((imu_current_time - imu_last_time) / 1.0e6 ) + 0.001f;

            imu_last_time = imu_current_time;
            
            return imu_error;
        }
};