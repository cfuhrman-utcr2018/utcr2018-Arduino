#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <stdlib.h>

const int Left_Motor = 10;
const int Right_Motor = 11;


ros::NodeHandle nh_arduino; // Initiated the node handle

void duty_left (const std_msgs::UInt16& duty_cycle_L){
  analogWrite(Left_Motor, duty_cycle_L.data); // accessign the data within the 
  // UInt16 wrapper
}

void duty_right (const std_msgs::UInt16& duty_cycle_R){
  analogWrite(Right_Motor, duty_cycle_R.data);
}

ros::Subscriber<std_msgs::UInt16> sub_D_L("Duty_Cycle_Left/command", duty_left);
ros::Subscriber<std_msgs::UInt16> sub_D_R("Duty_Cycle_Right/command", duty_right);


void setup() {
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);

  nh_arduino.initNode();
  nh_arduino.subscribe(sub_D_L);
  nh_arduino.subscribe(sub_D_R);
}

void loop() {
  nh_arduino.spinOnce();
}
