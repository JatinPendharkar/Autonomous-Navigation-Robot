#include <ArduinoHardware.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define mlf 4
#define mlb 5
#define mrf 6
#define mrb 7
#define enL 13 //Left Motor
#define enR 12 //Right Motor

#define encoderLpinA 3    // encoder Left
#define encoderLpinB 2 

#define encoderRpinA 20    // encoder Right
#define encoderRpinB 21

volatile long encoderLpos = 0;    // encoder Left
volatile long encoderRpos = 0;    // encoder Right

#define wheelRadius 0.06
#define wheelSeparation 0.23
#define MaxVelLinear 0.5     // meters per second
#define PwmAtMaxLinVel 210
#define PwmAtMinLinVel 120
#define MaxVelOmega MaxVelLinear/wheelRadius
//#define mapOmegaToPwm(omega) (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) + PwmAtMinLinVel)

#define L2RCompensation 1.0 //1.095  
#define AngCompensation 5
unsigned long startTime, stopTime;


inline int clampPwmToBoundry(int pwm){
  if(pwm<0) return 0;
  if(pwm>255) return 255;
  return pwm;
}


ros::NodeHandle nh;

double wl, wr;
//WHEEL ODOMETRY PUBLISHER
std_msgs::Int32 msg_odom_FL;
std_msgs::Int32 msg_odom_FR;

ros::Publisher chatter_FL("FL_sub", &msg_odom_FL);
ros::Publisher chatter_FR("FR_sub", &msg_odom_FR);


void moveMotor(int wlPwm, int wrPwm) {

  analogWrite(enL, clampPwmToBoundry(L2RCompensation*abs(wlPwm)));
  analogWrite(enR, clampPwmToBoundry(abs(wrPwm)/L2RCompensation));

  if (wlPwm > 0.0 && wrPwm > 0.0) {
    digitalWrite(mrf, HIGH);
    digitalWrite(mrb, LOW);
    digitalWrite(mlf, HIGH);
    digitalWrite(mlb, LOW);
  }
  else if (wlPwm < 0.0 && wrPwm < 0.0) {
    digitalWrite(mlf, LOW);
    digitalWrite(mlb, HIGH);
    digitalWrite(mrf, LOW);
    digitalWrite(mrb, HIGH);
  } 
  else if (wrPwm > 0.0 && wlPwm < 0.0) {
    digitalWrite(mrf, HIGH);
    digitalWrite(mrb, LOW);
    digitalWrite(mlf, LOW);
    digitalWrite(mlb, HIGH);
  }
  else if (wrPwm < 0.0 && wlPwm > 0.0) {
    digitalWrite(mlf, HIGH);
    digitalWrite(mlb, LOW);
    digitalWrite(mrf, LOW);
    digitalWrite(mrb, HIGH);
  }
  else {
    digitalWrite(mlf, LOW);
    digitalWrite(mlb, LOW);
    digitalWrite(mrf, LOW);
    digitalWrite(mrb, LOW);
  }
}

inline int mapOmegaToPwm(double omega) {
  if (omega > 0.0) return  (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) + PwmAtMinLinVel);
  if (omega < 0.0) return  (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) - PwmAtMinLinVel);
  return 0;
}

double calculateLeftOmega(double lin, double ang) {
  return (lin / wheelRadius) + ((ang*AngCompensation*wheelSeparation) / (2.0 * wheelRadius));
}


double calculateRightOmega(double lin, double ang) {
  return (lin / wheelRadius) - ((ang*AngCompensation*wheelSeparation) / (2.0 * wheelRadius));
}

inline double clampOmegaToBoundry(double omega) {
  if (omega > MaxVelOmega) return MaxVelOmega;
  if (omega < -MaxVelOmega) return -MaxVelOmega;
  return omega;
}

void cmdCb(const geometry_msgs::Twist& msg) {
  wl = calculateLeftOmega(msg.linear.x, msg.angular.z);
  wr = calculateRightOmega(msg.linear.x, msg.angular.z);
  moveMotor(mapOmegaToPwm(clampOmegaToBoundry(wl)),mapOmegaToPwm(clampOmegaToBoundry(wr)));

}

ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", &cmdCb);



void MotorInit() {
  pinMode(mlf, OUTPUT);
  pinMode(mlb, OUTPUT);
  pinMode(mrf, OUTPUT);
  pinMode(mrb, OUTPUT);

  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);

  pinMode(encoderLpinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoderLpinB, INPUT_PULLUP);
  

  pinMode(encoderRpinA, INPUT_PULLUP); 
  pinMode(encoderRpinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), doEncoderA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderRpinA), doEncoderC, CHANGE);

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
  msg_odom_FL.data=encoderLpos;
  msg_odom_FR.data=encoderRpos;
  chatter_FL.publish(&msg_odom_FL);
  chatter_FR.publish(&msg_odom_FR);

  nh.spinOnce();
  delay(300);

}

void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoderLpinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderLpinB) == LOW) {  
      encoderLpos = encoderLpos + 1;         // CW
    } 
    else {
      encoderLpos = encoderLpos - 1;         // CCW
    }
  }
 
}

// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoderRpinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderRpinB) == LOW) {  
      encoderRpos = encoderRpos - 1;         // CW
    } 
    else {
      encoderRpos = encoderRpos + 1;         // CCW
    }
  }
}
