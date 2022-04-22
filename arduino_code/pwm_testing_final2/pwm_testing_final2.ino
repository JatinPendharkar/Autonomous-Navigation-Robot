#include <ArduinoHardware.h>

#define mlf 4
#define mlb 5
#define mrf 6
#define mrb 7
#define enL 13 //Left Motor
#define enR 12 //Right Motor

#define encoderLpinA 21    // encoder Left
#define encoderLpinB 20 

#define encoderRpinA 2    // encoder Right
#define encoderRpinB 3

volatile long encoderLpos = 0;    // encoder Left
volatile long encoderRpos = 0;    // encoder Right

int testPwm;

#define wheelRadius 0.06
#define wheelSeparation 0.23

#define MaxVelLinear 0.5     // meters per second
#define PwmAtMaxLinVel 210
#define PwmAtMinLinVel 120
#define MaxVelOmega MaxVelLinear/wheelRadius
//#define mapOmegaToPwm(omega) (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) + PwmAtMinLinVel)

#define L2RCompensation 20  

unsigned long startTime, stopTime;


inline int clampPwmToBoundry(int pwm){
  if(pwm<0) return 0;
  if(pwm>255) return 255;
  return pwm;
}

void moveMotor(int wlPwm, int wrPwm) {

  analogWrite(enL, clampPwmToBoundry(L2RCompensation+abs(wlPwm)));
  analogWrite(enR, clampPwmToBoundry(abs(wrPwm)-L2RCompensation));

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


void initMotors() {
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
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), doEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), doEncoderRight, CHANGE);

}

void stopMotor() {
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, LOW);
}

void fetchPwmFromUser() {
  Serial.println("waiting for pwm input");

  while (Serial.available() <= 0){}

  testPwm = Serial.parseInt();
  startTime = millis();
  Serial.print("got time");
  Serial.println(testPwm);

}

inline int mapOmegaToPwm(double omega) {
  if (omega > 0.0) {
    return  (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) + PwmAtMinLinVel);

  }
  else if (omega < 0.0) {
    return  (int)(((omega) * (PwmAtMaxLinVel - PwmAtMinLinVel))/(MaxVelOmega) - PwmAtMinLinVel);

  }
  else {
    return 0;
  }
}

void setup() {
    initMotors();
    
    Serial.begin(9600);
    Serial.println("Serial Monitor Started");
    fetchPwmFromUser();
}

void loop() {

    
    
      if (encoderLpos < 195 && encoderRpos < 195) {
     
          moveMotor(testPwm, -testPwm);
          Serial.print("l: ");
          Serial.print(encoderLpos);
          Serial.print(" r: ");
          Serial.println(encoderRpos);
          Serial.println(testPwm);
      }
      else {
          stopTime = millis();
          Serial.print("time: ");
          Serial.println((stopTime-startTime)*0.001);
           Serial.print("l: ");
          Serial.print(encoderLpos);
          Serial.print(" r: ");
          Serial.println(encoderRpos);
          stopMotor();
          // reset encoder values for next test
          encoderLpos = 0;
          encoderRpos = 0;
          testPwm = 0;
          fetchPwmFromUser();
      }

    
   
  delay(100);
}

void doEncoderLeft() {
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

void doEncoderRight() {
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
