/*
 * rosserial subscriber to Joy node
 * for IGVC 2018 proof of concept
 */

#include <ros.h>
//#include <std_msgs.h>
//#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh; /* this instantiates the node handle*/


//void joydata (const sensor_msgs::Joy& joy){
//  if ((joy.buttons[0]) > 0.5){
//    digitalWrite(7, HIGH-digitalRead(7));
//  }
//  if ((joy.buttons[1]) > 0.5){
//    digitalWrite(6, HIGH-digitalRead(6));
//  }
//}
//ros::Subscriber<sensor_msgs::Joy> sub("joy", joydata);

void joydata (const std_msgs::Float64& velocity_left){
  if ((velocity_left.data) > 0.9){
    digitalWrite(7, HIGH-digitalRead(7));
  }
  if ((velocity_left.data) > 0.9){
    digitalWrite(6, HIGH-digitalRead(6));
  }
  
}

ros::Subscriber<std_msgs::Float64> sub("left_velocity_controller/command", joydata);

void setup()
{ 
  digitalWrite(6, HIGH);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{  
  nh.spinOnce();
  //delay(0.1);
}


