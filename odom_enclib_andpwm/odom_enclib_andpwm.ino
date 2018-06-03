#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

Encoder right(2, 13);
Encoder left(3, 12); 

std_msgs::Int16 left_ticks_number;
std_msgs::Int16 right_ticks_number;
// Init the publishers
ros::Publisher left_pub("left_ticks", &left_ticks_number);
ros::Publisher right_pub("right_ticks", &right_ticks_number);

const int Left_Motor = 10;
const int Right_Motor = 9;
const int Green_Light = 5;

long positionLeft  = -999;
long positionRight = -999;

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
  nh_arduino.advertise(left_pub);
  nh_arduino.advertise(right_pub);
}

void loop() {
  long newLeft, newRight;
  newLeft = left.read();
  newRight = right.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    left_ticks_number.data = newLeft;
    right_ticks_number.data = newRight;
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  left_pub.publish(&left_ticks_number);
  right_pub.publish(&right_ticks_number);
 
  nh_arduino.spinOnce();
}
