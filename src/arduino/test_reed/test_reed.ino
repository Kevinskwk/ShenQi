// ROS
#include <ros.h>
#include <std_msgs/ByteArray.h>

ros::NodeHandle nh;

// Setting Variables
const int reedRow[10] = {35, 34, 33, 32, 31, 30, 29, 28, 27, 26};
const int reedCol[9] = {24, 25, 23, 36, 37, 38, 39, 40, 41};
const char num2letter[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

int reeds[90];
// If 9x10 array is used
// int reeds[9][10];


void setup(){
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  for (int i=0; i<10; ++i){
    pinMode(reedRow[i], INPUT);
  }
  for (int i=0; i<9; ++i){
    pinMode(reedCol[i], OUTPUT);
    digitalWrite(reedCol[i], LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  checkReed();
  delay(800);
  Serial.println("-----");
}

void checkReed(){
  for (int a=0; a<3; ++a){
    digitalWrite(reedCol[a], HIGH);
    //Serial.println("Testing row ");
    //Serial.print(a, DEC);
    //Serial.println(' ');
    //delay(100);
    for (int b=0; b<10; ++b){
      int state = digitalRead(reedRow[b]);
      if (state == 1) {
        Serial.print(num2letter[a]);
        Serial.println(b+1);
      }
      //Serial.print("Testing: ");
      //Serial.print(num2letter[a]);
      //Serial.print(b+1);
      //Serial.print(": ");
      //Serial.print(state);
      //Serial.print(", ");
      //delay(10);
      //Serial.println(b);
      // reeds[a + b] = state;
      //Serial.println(reeds[a-32+(b-22)]);
// If 9x10 array is used
//      reeds[a-32][b-22] = state;
//      Serial.println(reeds[a-32][b-22]);
      //delay(10);
    }
    //Serial.println();
    digitalWrite(reedCol[a], LOW);
    delay(10);
  }
}

void Hbot(){
  
}
