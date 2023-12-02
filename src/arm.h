#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

#define GRIPPER_PIN 10
#define LIFT_PIN 9

Servo gripper;
Servo lift;

VL53L0X sensor;


class Arm{
    public:
        void initial(void){
            gripper.attach(GRIPPER_PIN);
            lift.attach(LIFT_PIN);
            gripper.write(40);
            lift.write(173);

            Wire1.begin();
            sensor.setTimeout(500);
            sensor.setBus(&Wire1);
            if (!sensor.init())
            {
                Serial.println("Failed to detect and initialize sensor!");
                while (1) {}
            }

            // Start continuous back-to-back mode (take readings as
            // fast as possible).  To use continuous timed mode
            // instead, provide a desired inter-measurement period in
            // ms (e.g. sensor.startContinuous(100)).
            sensor.startContinuous();
        }

        uint16_t read_tof(void){
            uint16_t distance = sensor.readRangeContinuousMillimeters();
            return distance;
        }

        void lift_up(void){
            lift.write(60);
        }

        void grip(void){
            gripper.write(15);

        }
        

};
