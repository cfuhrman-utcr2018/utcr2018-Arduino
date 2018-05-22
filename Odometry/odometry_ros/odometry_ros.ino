//=========================HEADER=============================================================
/*
   Dual LS7366 Quadrature Counter Test Code
   AUTHOR: Jason Traud
   DATE: June 22, 2013
   
   This is a simple test program to read encoder counts
   collected by the LS7366 breakout board. The counts are
   then displayed in the Arduino's serial monitor at a 
   baud rate of 9600
   
   Hardware: Arduino Uno R3
   Powered 
   
   LS7366 Breakout    -------------   Arduino
   -----------------                    -------
            MOSI   -------------------   SDO (D11)
            MISO   -------------------   SDI (D12)
            SCK    -------------------   SCK (D13)
            SS1    -------------------   SS1 (D7)
            SS2    -------------------   SS2 (D8)
            GND    -------------------   GND
            VDD    -------------------   VCC (5.0V)
      
   License: CCAv3.0 Attribution-ShareAlike (http://creativecommons.org/licenses/by-sa/3.0/)
   You're free to use this code for any venture. Attribution is greatly appreciated. 

//============================================================================================
*/

// ===========================================================================================
/* ROS code written by Connor Fuhrman in 2018 for the IGVC. Code written at The Citadel The 
 *  Miliary COllege of South Corolina for the purpose of competing in the IGVC.
 */
// ===========================================================================================

// Inclde the standard Arduino SPI Library, please ensure the SPI pins are
// connected properly for your Arduino version
#include <SPI.h>

// Include the ROS header file
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <stdlib.h>
// Initialize ROS stuff
// Init the variabels to be published
std_msgs::Int16 left_ticks_number;
std_msgs::Int16 right_ticks_number;
// Init the publishers
ros::Publisher left_pub("left_ticks", &left_ticks_number);
ros::Publisher right_pub("right_ticks", &right_ticks_number);


// Init the ROS node
ros::NodeHandle nh_arduino;


// Slave Select pins for encoders 1 and 2
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;

// Initialize the motor PWM pins
const int Left_Motor = 10;
const int Right_Motor = 9;
const int Green_Light = 5;

void duty_left (const std_msgs::UInt8& duty_cycle_L){
  analogWrite(Left_Motor, duty_cycle_L.data); // accessign the data within the 
  // UInt16 wrapper
}

void duty_right (const std_msgs::UInt8& duty_cycle_R){
  analogWrite(Right_Motor, duty_cycle_R.data);
}

// Init the subscribers
ros::Subscriber<std_msgs::UInt8> sub_D_L("Duty_Cycle_Left", duty_left);
ros::Subscriber<std_msgs::UInt8> sub_D_R("Duty_Cycle_Right", duty_right);
//ros::Subscriber<std_msgs::UInt8> sub_L("Light", light);

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
}


void setup() {
  
  nh_arduino.initNode();
  // Make PWM and light pins outputs
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);
  pinMode(Green_Light, OUTPUT);
  // Start the publishing and subscribing
  nh_arduino.advertise(left_pub);
  nh_arduino.advertise(right_pub);
  nh_arduino.subscribe(sub_D_L);
  nh_arduino.subscribe(sub_D_R);
//  nh_arduino.subscribe(sub_L);
  
  initEncoders();       //Serial.println("Encoders Initialized...");  
  clearEncoderCount();  //Serial.println("Encoders Cleared...");
}

void loop() {
 //delay(500);
 
 // Retrieve current encoder counters
 encoder1count = readEncoder(1); 
 encoder2count = readEncoder(2);

 // Assign encoder counts to ROS ticks topics
 left_ticks_number.data = encoder1count;
 right_ticks_number.data = encoder2count;

 left_pub.publish(&left_ticks_number);
 right_pub.publish(&right_ticks_number);

 nh_arduino.spinOnce();
 //Serial.print("Enc1: "); Serial.print(encoder1count); Serial.print(" Enc2: "); Serial.println(encoder2count); 
}
