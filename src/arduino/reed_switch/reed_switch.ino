#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

// shape of grid
#define W 9
#define H 10

// Reed switch
const int reedRow[10] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
const int reedCol[9] = {23, 25, 24, 38, 37, 36, 41, 40, 39};
// const char num2letter[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

// ROS
ros::NodeHandle nh;
std_msgs::Int8MultiArray reedsMsg;
ros::Publisher reedPub("sensor", &reedsMsg);

void setup() {
  // Reed switches
  for (int i = 0; i < 10; ++i) {
    pinMode(reedRow[i], INPUT);
  }
  for (int i = 0; i < 9; ++i) {
    pinMode(reedCol[i], OUTPUT);
    digitalWrite(reedCol[i], LOW);
  }

  // ROS
  reedsMsg.data = (std_msgs::Int8MultiArray::_data_type *)malloc(sizeof(short)*90);
  reedsMsg.data_length = 90;
  nh.initNode();
  nh.advertise(reedPub);

  delay(250);
}

void loop() {
  checkReed();
  nh.spinOnce();
  delay(50);
}

void checkReed(){
  for (int i = 0; i < 9; ++i){
    digitalWrite(reedCol[i], HIGH);
    delay(10);
    for (int j = 0; j < 10; ++j){
      reedsMsg.data[j * W + i] = digitalRead(reedRow[j]);
      /*if (state == 1) {
        Serial.print(num2letter[a]);
        Serial.println(b+1);
      }*/
    }
    digitalWrite(reedCol[i], LOW);
  }
  reedPub.publish(&reedsMsg);
}