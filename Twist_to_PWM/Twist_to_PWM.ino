#include <ros.h>
#include<geometry_msgs/Twist.h>
#include <stdlib.h>

using namespace ros;
using namespace geometry_msgs;

const int Left_Motor = 9; // pin number for left motor PWM
const int Right_Motor = 10; // pin number for right motor PWM
const float W = 0.68; // meters
const float Max_Speed = 1.0; // meters/second - the maximum speed of the robot
int R_PWM = 0; // PWM number from 0 to 255 for the Right motor
int L_PWM = 0; // PWM number from 0 to 255 for the Left motor

NodeHandle nh_arduino; // Initiating the node

void velocity (const Twist& twist){
  float vx = twist.linear.x;
  float wz = twist.angular.z;

    if ( vx > 0 ){
      R_PWM = (255/Max_Speed)*(vx+(wz*W/2));
      L_PWM = (255/Max_Speed)*(vx-(wz*W/2));  
    }
     if ( vx <0 ){
      R_PWM = (154/Max_Speed)*(vx-(wz*W/2));
      L_PWM = (154/Max_Speed)*(vx+(wz*W/2));
    }
  
    analogWrite(Left_Motor, L_PWM);
    analogWrite(Right_Motor, R_PWM);
    
// Debugging purposes
    Serial.print("Left PWM is: ");
    Serial.print(L_PWM);
    Serial.print("Right PWM is: ");
    Serial.print(R_PWM);
// end debug section  
}

Subscriber<Twist> Cmd_vel_sub("cmd_vel", velocity);

void setup() {
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);

  nh_arduino.initNode();
  nh_arduino.subscribe(Cmd_vel_sub);
}

void loop() {
  nh_arduino.spinOnce();
}
