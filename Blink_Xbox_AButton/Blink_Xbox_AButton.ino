/*
 * rosserial subscriber to Joy node
 * for IGVC 2018 proof of concept
 */

#include <ros.h>
//#include <std_msgs.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle nh; /* this instantiates the node handle*/

void joydata (const sensor_msgs::Joy& joy){
  if ((joy.buttons[0]) > 0.9){
  digitalWrite(13, 1-digitalRead(13));
  }
}
ros::Subscriber<sensor_msgs::Joy> sub("joy", joydata);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{  
  nh.spinOnce();
  delay(1);
}
