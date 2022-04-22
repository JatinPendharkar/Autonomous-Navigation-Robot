#include <ArduinoHardware.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define m11 3
#define m12 4
#define m21 8
#define m22 9
#define enA 2
#define enB 10

// ########################### WHEEL PARAMETERS ##############################
double wheelRadius = 0.05;
double wheelSeparation = 0.20;

double wl = 0.0;
double wr = 0.0;

ros::NodeHandle nh;

double speedAng = 0.0;
double speedLin = 0.0;

void MotorL(double wlPwm) {
  analogWrite(enA, (int) wlPwm);
  if (wlPwm > 0.0) {
    digitalWrite(m11, HIGH);
    digitalWrite(m12, LOW);
  }
  else if (wlPwm < 0.0) {
    digitalWrite(m11, LOW);
    digitalWrite(m12, HIGH);
  } 

  
  else if (wlPwm == 0.0) {
    digitalWrite(m11, LOW);
    digitalWrite(m12, LOW);
  } 
  
}

void MotorR(double wlPwm) {
  analogWrite(enB, (int) wlPwm);
  if (wlPwm > 0.0) {
    digitalWrite(m21, HIGH);
    digitalWrite(m22, LOW);
  }
  else if (wlPwm < 0.0) {
    digitalWrite(m21, LOW);
    digitalWrite(m22, HIGH);
  } 
  else if (wlPwm == 0.0) {
    digitalWrite(m21, LOW);
    digitalWrite(m22, LOW);
  } 

  
  
  
}

void cmdCb(const geometry_msgs::Twist& msg) {
  speedAng = ((msg.angular.z*2.5*wheelSeparation) / (2.0 * wheelRadius));
  speedLin = (msg.linear.x / wheelRadius);
  wr = speedLin + speedAng;
  wl = speedLin - speedAng;
  
  MotorL(wl * 10);
  MotorR(wr * 10);

}

ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", &cmdCb);

void MotorInit() {
  pinMode(m11, OUTPUT);
  pinMode(m12, OUTPUT);
  pinMode(m21, OUTPUT);
  pinMode(m22, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
}

void setup()
{
  MotorInit();
  nh.initNode();
  nh.subscribe(cmdSub);
}




void loop()
{

  nh.spinOnce();
  delay(300);

}
