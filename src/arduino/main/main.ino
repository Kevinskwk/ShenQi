#include <Stepper.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Empty.h>

// change this to fit the number of steps per revolution for your motor
#define STEPS_PER_RESOLUTION 200
// shape of grid
#define W 9
#define H 10

// Limit switch pins
#define Lswitch_x 8
#define Lswitch_y 9
// Magnet pin
#define magnet 7

// Reed switch
const short reedRow[10] = {35, 34, 33, 32, 31, 30, 29, 28, 27, 26};
const short reedCol[9] = {24, 25, 23, 36, 37, 38, 39, 40, 41};
// const char num2letter[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

// initialize the stepper library:
Stepper myStepperL(STEPS_PER_RESOLUTION, 10, 11);
Stepper myStepperR(STEPS_PER_RESOLUTION, 13, 12);

// HBot variables
// TODO: add the coordinate table
const unsigned int table_x[11] = {0, 3625, 7250, 14500, 21750, 29000, 36250,
                                  43500, 50750, 58000, 65250};
const unsigned int table_y[26] = {0, 7250, 14500, 21750, 29000, 36250, 43500,
                                  50750, 58000, 65250, 0, 4350, 8700, 13050,
                                  17400, 21750, 26100, 30450, 34800, 39150,
                                  43500, 47850, 52200, 56550, 60900, 65250};
long x, y;
long coordinates[2] = {0, 0}; // Initialize with home position
bool debounce = false;

// ROS stuff
void movementCb( const std_msgs::Int8MultiArray& msg) {
  switch (msg.data[2]) {
    case 0:
      digitalWrite(magnet, LOW);
      break;
    case 1:
      digitalWrite(magnet, HIGH);
      break;
    case 2:
      home();
      return;
    default:
      digitalWrite(magnet, LOW);
      break;
  }
  x = table_x[msg.data[0]] - coordinates[0];
  y = table_y[msg.data[1]] - coordinates[1];
  coordinates[0] = table_x[msg.data[0]];
  coordinates[1] = table_y[msg.data[1]];
  Hbot();
}

ros::NodeHandle nh;
std_msgs::Int8MultiArray reedsMsg;
std_msgs::Empty feedbackMsg;
ros::Subscriber<std_msgs::Int8MultiArray> movementSub("movement", &movementCb);
ros::Publisher reedPub("sensor", &reedsMsg);
ros::Publisher feedbackPub("feedback", &feedbackMsg);


void setup() {
  // Digital pins
  pinMode(Lswitch_x, INPUT_PULLUP);
  pinMode(Lswitch_y, INPUT_PULLUP);

  pinMode(magnet, OUTPUT);
  digitalWrite(magnet, LOW);

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
  nh.subscribe(movementSub);
  nh.advertise(reedPub);
  nh.advertise(feedbackPub);

  delay(250);
  home();
}

void loop() {
  checkReed();
  nh.spinOnce();
  delay(100);
}

void checkReed(){
  for (int i = 0; i < 9; ++i){
    digitalWrite(reedCol[i], HIGH);
    for (int j = 0; j < 10; ++j){
      reedsMsg.data[j * W + i] = digitalRead(reedRow[j]);
      /*if (state == 1) {
        Serial.print(num2letter[a]);
        Serial.println(b+1);
      }*/
    }
    digitalWrite(reedCol[i], LOW);
    delay(10);
  }
  reedPub.publish(&reedsMsg);
}


void Hbot() {
  // moving the hbot
  // movement cases
  // x-direction
  if (x != 0 && y == 0) {
    // right
    if (x > 0){
      for (long s = 0; s < x; ++s) {
        myStepperL.step(1);
        myStepperR.step(1);
      }
    }
    // left
    else {
      for (long s = 0; s > x; --s) {
        myStepperL.step(-1);
        myStepperR.step(-1);
      }
    }
  }
  
  // y-direction
  else if (x == 0 && y != 0) {
    // up
    if (y > 0){
      for (long s = 0; s < y; ++s) {
        myStepperL.step(-1);
        myStepperR.step(1);
      }    
    }
    // down
    else {
      for (long s = 0; s > y; --s) {
        myStepperL.step(1);
        myStepperR.step(-1);
      }   
    }    
  }
  
  // diagonal
  else if (abs(x) == abs(y) != 0) {
    // top right
    if (x > 0 && y > 0) {
      myStepperR.step(2 * x);
    }
    // top left
    else if (x < 0 && y > 0) {
      myStepperL.step(2 * x);
    }
    // bottom right
    else if (x > 0 && y < 0) {
      myStepperL.step(2 * x);
    }
    // bottom left
    else {
      myStepperR.step(2 * x);
    }
  }
  x = 0L;
  y = 0L;
  delay(200);
  feedbackPub.publish(&feedbackMsg);
}

void home() {
  // off the magnet first
  digitalWrite(magnet, LOW);
  // x-axis
  for (int x=0; x < 100000L; ++x){
    if (digitalRead(Lswitch_x) == HIGH){
      if (debounce) break;
      else {
        debounce = true;
        delay(5);
      }
    }
    else {
      debounce = false;
      myStepperL.step(-1);
      myStepperR.step(-1);
    }
  }
  // Serial.println("Done x-homing");
  delay(50);

  // y-axis
  for (long y = 0; y < 100000L; ++y){
    if (digitalRead(Lswitch_y) == HIGH){
      if (debounce) break;
      else {
        debounce = true;
        delay(5);
      }
    }
    else {
      debounce = false;
      myStepperL.step(-1);
      myStepperR.step(1);
    }
  }
  // Serial.println("Done y-homing");
  // set to home position
  coordinates[0] = 0L;
  coordinates[1] = 0L;
}
