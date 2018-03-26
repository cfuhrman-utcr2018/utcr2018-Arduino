#include <ros.h>
#include <std_msgs/UInt16.h>
#include <stdlib.h>

const int Left_Motor = 9;
const int Right_Motor = 10;
const int Green_Light = 2;

ros::NodeHandle nh_arduino; // Initiated the node handle

void duty_left (const std_msgs::UInt16& duty_cycle_L){
  analogWrite(Left_Motor, duty_cycle_L.data); // accessign the data within the 
  // UInt16 wrapper
}

void duty_right (const std_msgs::UInt16& duty_cycle_R){
  analogWrite(Right_Motor, duty_cycle_R.data);
}

void light (const std_msgs::UInt16& light){
  if( light.data == 1 ){
    digitalWrite(Green_Light, HIGH);
  }
  else digitalWrite(Green_Light, LOW);
}

ros::Subscriber<std_msgs::UInt16> sub_D_L("Duty_Cycle_Left", duty_left);
ros::Subscriber<std_msgs::UInt16> sub_D_R("Duty_Cycle_Right", duty_right);
ros::Subscriber<std_msgs::UInt16> sub_L("Light", light);


void setup() {
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);
  pinMode(Green_Light, OUTPUT);

  nh_arduino.initNode();
  nh_arduino.subscribe(sub_D_L);
  nh_arduino.subscribe(sub_D_R);
  nh_arduino.subscribe(sub_L);
}

void loop() {
  nh_arduino.spinOnce();
}
