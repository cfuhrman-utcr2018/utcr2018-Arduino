#include <ros.h>
#include <std_msgs/Float64.h>

int Left_Motor = 10;
int Right_Motor = 11;


ros::NodeHandle nh_arduino; // Initiated the node handle

void duty_left (const std_msgs::Float64& duty_cycle_L){
  analogWrite(Left_Motor, duty_left);
}
/*
void duty_right (const std_msgs::Float64& duty_cycle_R){
  analogWrite(Right_Motor, duty_right);
}
*/
ros::Subscriber<std_msgs::Float64> sub_D_L("Duty_Cycle_Left/command", duty_left);
//ros::Subscriber<std_msgs::Float64> sub_D_R("Duty_Cycle_Right/command", duty_right);


void setup() {
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);

  nh_arduino.initNode();
  nh_arduino.subscribe(sub_D_L);
  //nh_arduino.subscribe(sub_D_R);
}

void loop() {
  nh_arduino.spinOnce();

}