#include <Stepper.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;

// variables
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
unsigned int x = 0;
unsigned int y = 0;
unsigned int coordinates[2] = {0, 0};
const unsigned int k = 14500;

void messageCb( const std_msgs::UInt16MultiArray& msg){
  x = msg.data[0] - coordinates[0];
  y = msg.data[1] - coordinates[1];
  coordinates[0] = msg.data[0];
  coordinates[1] = msg.data[1];
  //Hbot();
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("movement", &messageCb);

// initialize the stepper library on pins 8 through 11:
Stepper myStepperL(stepsPerRevolution, 10, 11);
Stepper myStepperR(stepsPerRevolution, 13, 12);

void setup() {
  // put your setup code here, to run once:
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read directions from rpi(serial)
  nh.spinOnce();
  delay(250);
}


void Hbot(){
  // moving the hbot
  // movement cases
  // x-direction
  if (x != 0 && y == 0){
    // right
    if (x > 0){
      for (uint16_t s=0; s<k*abs(x); s++){
        myStepperL.step(1);
        myStepperR.step(1);
      }
      //myStepperL.step(k*x);
      //myStepperR.step(k*x);
      delay(1200);
    }
    // left
    else {
      for (uint16_t s=0; s<k*abs(x); s++){
        myStepperL.step(-1);
        myStepperR.step(-1);
      }
      delay(1200);
    }
  }
  
  // y-direction
  else if (x == 0 && y != 0){
    // up
    if (y > 0){
      for (uint16_t s=0; s<k*abs(y); s++){
        myStepperL.step(-1);
        myStepperR.step(1);
      }
      delay(1200);    
    }
    // down
    else {
      for (uint16_t s=0; s<k*abs(y); s++){
        myStepperL.step(1);
        myStepperR.step(-1);
      }
      delay(1200);    
    }    
  }
  
  // diagonal
  else if (abs(x) == abs(y) != 0){
    // top right
    if (x > 0 && y > 0){
      myStepperR.step(2*k*abs(x));
      delay(1200);
    }
    // top left
    else if (x < 0 && y > 0){
      myStepperL.step(-2*k*abs(x));
      delay(1200);
    }
    // bottom right
    else if (x > 0 && y < 0){
      myStepperL.step(2*k*abs(x));
      delay(1200);
    }
    // bottom left
    else {
      myStepperR.step(-2*k*abs(x));
      delay(1200);
    }   
  }
  
  // stationary
  else {
    delay(1200);
  }
}
