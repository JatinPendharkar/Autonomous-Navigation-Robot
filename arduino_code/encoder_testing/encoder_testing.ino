void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

#include <ArduinoHardware.h>

#define encoderFLA 6
#define encoderFLB 7

#define encoderFRA 12
#define encoderFRB 13

// VARIABLE
int pulses_FL = 0;                              
bool pulsesChanged_FL = false; 
int pulses_FR = 0;                              
bool pulsesChanged_FR = false;

void A_CHANGE();

void B_CHANGE();

void setup()
{
    pinMode(encoderFLA, INPUT);
    pinMode(encoderFLB, INPUT);
    pinMode(encoderFRA, INPUT);
    pinMode(encoderFRB, INPUT);
    attachInterrupt(0, A_CHANGE, CHANGE);
    attachInterrupt(0, B_CHANGE, CHANGE);
    Serial.begin(9600);
    Serial.println("Serial Monitor Started");
}

void loop()
{
    if (pulsesChanged_FL) {
        pulsesChanged_FL = false;
    }
    
    if (pulsesChanged_FR) {
        pulsesChanged_FR = false;
    }
    Serial.print("left encoder");
    Serial.println(pulses_FL);
    Serial.print("right encoder");
    Serial.println(pulses_FR);
    delay(300);
}

void A_CHANGE(){                                 
  if( !digitalRead(encoderFLB)) {
    if ( !digitalRead(encoderFLA)) {
      // A fell, B is low
      pulses_FL--; // moving reverse
    } else {
      // A rose, B is low
      pulses_FL++; // moving forward
    }
  }
  pulsesChanged_FL = true;
}

void A_CHANGE()
{                                 
  if( !digitalRead(encoderFLB)) {
    if ( !digitalRead(encoderFLA)) {
      // A fell, B is low
      pulses_FL--; // moving reverse
    } else {
      // A rose, B is low
      pulses_FL++; // moving forward
    }
  }
  pulsesChanged_FL = true;
}
