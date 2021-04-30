/*
 * Arduino code for controling the Hbot X-Y motion system. Receives
 * Int8MultiArray commands and publishes feedback debug messages.
 * Tested on Arduino Nano
 */

#include <Stepper.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

// change this to fit the number of steps per revolution for your motor
#define STEPS_PER_RESOLUTION 200

// Stepper motor pins
#define LDir 9
#define LStep 10
#define RDir 11
#define RStep 12
// Limit switch pins
#define Lswitch_x 7
#define Lswitch_y 6
// Magnet pin
#define magnet 8

// initialize the stepper library:
Stepper myStepperL(STEPS_PER_RESOLUTION, LDir, LStep);
Stepper myStepperR(STEPS_PER_RESOLUTION, RDir, RStep);

// HBot variables
const long table_x[11] = {0L, 4000L, 9050L, 16300L, 23550L, 30800L, 38050L,
                                  45300L, 52550L, 59800L, 67050L};
const long table_y[26] = {0L, 7250L, 14500L, 21750L, 29000L, 36250L, 43500L,
                          50750L, 58000L, 65250L, 0L, 4350L, 8700L, 13050L,
                          17400L, 21750L, 26100L, 30450L, 34800L, 39150L,
                          43500L, 47850L, 52200L, 56550L, 60900L, 65250L};
long target_x = 0L;
long target_y = 0L;
long curr_x = 0L;
long curr_y = 0L;
bool debounce = false;
bool moving = false;

// ROS stuff
void movementCb(const std_msgs::Int8MultiArray& msg) {
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
  target_x = table_x[msg.data[0]] - curr_x;
  target_y = table_y[msg.data[1]] - curr_y;
  curr_x = table_x[msg.data[0]];
  curr_y = table_y[msg.data[1]];
  Hbot();
}

ros::NodeHandle nh;
std_msgs::Empty feedbackMsg;
std_msgs::Int32 debugMsg;
ros::Subscriber<std_msgs::Int8MultiArray> movementSub("movement", &movementCb);
ros::Publisher feedbackPub("feedback", &feedbackMsg);
ros::Publisher debugPub("debug", &debugMsg);

void setup() {
  // Digital pins
  pinMode(Lswitch_x, INPUT_PULLUP);
  pinMode(Lswitch_y, INPUT_PULLUP);

  pinMode(magnet, OUTPUT);
  digitalWrite(magnet, LOW);

  // ROS
  nh.initNode();
  nh.subscribe(movementSub);
  nh.advertise(feedbackPub);
  nh.advertise(debugPub);

  delay(250);
  home();
}

void loop() {
  nh.spinOnce();
  delay(100);
}

void Hbot() {
  moving = true;
  // moving the hbot
  debugMsg.data = target_x;
  debugPub.publish(&debugMsg);
  // movement cases
  // diagonal
  if (abs(target_x) == abs(target_y) != 0) {
    // down right
    if (target_x > 0 && target_y > 0) {
      for (long s = 0; s < target_x; ++s) {
        myStepperL.step(2);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    // down left
    else if (target_x < 0 && target_y > 0) {
      for (long s = 0; s > target_x; --s) {
        myStepperR.step(-2);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    // up right
    else if (target_x > 0 && target_y < 0) {
      for (long s = 0; s < target_x; ++s) {
        myStepperR.step(2);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    // up left
    else {
      for (long s = 0; s > target_x; --s) {
        myStepperL.step(-2);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
  }

  // x,y-direction
  else {
    // right
    if (target_x > 0){
      for (long s = 0; s < target_x; ++s) {
        myStepperL.step(1);
        myStepperR.step(1);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    // left
    else {
      for (long s = 0; s > target_x; --s) {
        myStepperL.step(-1);
        myStepperR.step(-1);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    delay(50);
    // down
    if (target_y > 0){
      for (long s = 0; s < target_y; ++s) {
        myStepperL.step(1);
        myStepperR.step(-1);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
    // up
    else {
      for (long s = 0; s > target_y; --s) {
        myStepperL.step(-1);
        myStepperR.step(1);
        if (s % 1000 == 0) nh.spinOnce();
      }
    }
  }
  target_x = 0L;
  target_y = 0L;
  feedbackPub.publish(&feedbackMsg);
  nh.spinOnce();
  delay(100);
  moving = false;
}

void home() {
  // off the magnet first
  digitalWrite(magnet, LOW);
  // x-axis
  for (long s = 0; s < 100000L; ++s){
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
    if (s % 1000 == 0) nh.spinOnce();
    if (moving) break;
  }
  // Serial.println("Done x-homing");
  delay(50);

  // y-axis
  for (long s = 0; s < 100000L; ++s){
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
    if (s % 1000 == 0) nh.spinOnce();
    if (moving) break;
  }
  // Serial.println("Done y-homing");
  // set to home position
  curr_x = 0L;
  curr_y = 0L;
}
