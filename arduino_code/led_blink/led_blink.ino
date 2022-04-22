#include <ArduinoHardware.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define m11 5
#define m12 4
#define m21 7
#define m22 6

#define encoder0PinA 18    // encoder 1
#define encoder0PinB 19 

#define encoder1PinA 20    // encoder 2
#define encoder1PinB 21

unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

// VARIABLE
int pulses_FL;                              
bool pulsesChanged_FL = false; 
int pulses_FR;                              
bool pulsesChanged_FR = false;

// ########################### WHEEL PARAMETERS ##############################
#define wheelRadius 0.045
#define wheelSeparation 0.245

double wl = 0.0;
double wr = 0.0;

ros::NodeHandle nh;


//WHEEL ODOMETRY PUBLISHER
std_msgs::Int32 msg_odom_FL;
std_msgs::Int32 msg_odom_FR;

ros::Publisher chatter_FL("FL_sub", &msg_odom_FL);
ros::Publisher chatter_FR("FR_sub", &msg_odom_FR);


void moveMotor(const double& wlPwm, const double wrPwm) {
  if (wlPwm > 0.0 && wrPwm > 0.0) {
    
    digitalWrite(m21, HIGH);
    digitalWrite(m22, LOW);
    digitalWrite(m11, HIGH);
    digitalWrite(m12, LOW);
    
    
    
  }
  else if (wlPwm < 0.0 && wrPwm < 0.0) {
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
    
    digitalWrite(m21, LOW);
    digitalWrite(m22, HIGH);
  } 
  
  else if (wrPwm < 0.0 && wlPwm > 0.0) {
    digitalWrite(m21, HIGH);
    digitalWrite(m22, LOW);
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
  }
  else if (wrPwm > 0.0 && wlPwm < 0.0) {
    digitalWrite(m11, HIGH);
    digitalWrite(m12, LOW);
    digitalWrite(m21, LOW);
    digitalWrite(m22, HIGH);
  }
  else {
    digitalWrite(m11, LOW);
    digitalWrite(m12, LOW);
    digitalWrite(m21, LOW);
    digitalWrite(m22, LOW);
  }
  
  
}



void cmdCb(const geometry_msgs::Twist& msg) {
  
  wr = (msg.linear.x / wheelRadius) + ((msg.angular.z*2.5*wheelSeparation) / (2.0 * wheelRadius));
  wl = (msg.linear.x / wheelRadius) - ((msg.angular.z*2.5*wheelSeparation) / (2.0 * wheelRadius));
  moveMotor(wl,wr);

}

ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", &cmdCb);



void MotorInit() {
  pinMode(m11, OUTPUT);
  pinMode(m12, OUTPUT);
  pinMode(m21, OUTPUT);
  pinMode(m22, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
}

void setup()
{
  MotorInit();
  nh.initNode();
  nh.advertise(chatter_FL);
  nh.advertise(chatter_FR);
  nh.subscribe(cmdSub);
}




void loop()
{
  msg_odom_FL.data=encoder0Pos;
  msg_odom_FR.data=encoder1Pos;
  chatter_FL.publish(&msg_odom_FL);
  chatter_FR.publish(&msg_odom_FR);

  nh.spinOnce();
  delay(300);

}

void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
}


// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}
