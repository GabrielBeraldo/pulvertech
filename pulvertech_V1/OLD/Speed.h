//#define debugSpeed
//test
//#include "Arduino.h"
//#include "wiring_private.h"
//#include "pins_arduino.h"
//#include <Wire.h>

#define sensorInterrupt 0  // 0 = digital pin 2
#define SensorIn 2 //A0
#define timeoutSpeed 500
#define SpeedCalibrateButton A3
#define SpeedLed 5
#define SpeedLedBlink 420
#define N_MinSpd 20

//////////////////////////////////////////////////////////////////////////////////////
//////initialization of vars and functions///////////////////////////////////////////

float historySpeed[10];
double MeterPP = 0; //need to read from eeprom
double LastKMPH = 0;
volatile long pulseCountSpeed;
volatile long PulseDurationSpeed = 0;
unsigned long oldTimeSpeed = 0;
unsigned long MillisTimeoutSpeed = 0;
bool SpeedLedState=false;
int n=0;
bool LastState = false;

void SpeedSetup();
double ReadSpeed();
float SpeedAverage(float, int);
double SpeedCalibrate();
double SpeedCalibrateTimed();
bool MinSpeedTester(double, int);

void pulseCounterSpeed();
void PulseTime();

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void SpeedSetup()
{
    pinMode(SensorIn, INPUT);
    pinMode(SpeedCalibrateButton, INPUT);
    pinMode(SpeedLed, OUTPUT);

    for(int i = 0; i < 10; i++) historySpeed[i] = 0;

    attachInterrupt(sensorInterrupt, PulseTime, CHANGE);
}




double ReadSpeed(double MPP)
{

    /*double PulseTime=pulseIn(SensorIn, HIGH);
    if(PulseTime<0) PulseTime*=-1;*/
    double KMPH = 0;
    if(PulseDurationSpeed > 0)
        KMPH = ((MPP) / PulseDurationSpeed) * 3.6;



    if(KMPH == LastKMPH)
    {
        if( millis() - MillisTimeoutSpeed >= timeoutSpeed) KMPH = PulseDurationSpeed = 0;
    }
    else
    {
        MillisTimeoutSpeed = millis();
        LastKMPH = KMPH;
    }


#ifdef debugSpeed
    Serial.print("Speed:");
    Serial.print(KMPH);
    Serial.print(" PulseTime:");
    Serial.println(PulseDurationSpeed);

#endif

	if(KMPH>100) KMPH=0;
	else if(KMPH<0) KMPH=0;

    return(KMPH);
}

float SpeedAverage(float Spd, int n)
{

    float avrg = 0;
    if(Spd <= 0) Spd = 0;

    for(int i = 1; i < n; i++) historySpeed[i - 1] = historySpeed[i];
    historySpeed[n - 1] = Spd;
    for(int i = 0; i < n; i++) avrg += historySpeed[i];
    historySpeed[n - 1] = avrg / n;

#ifdef debugFluxAverage
    for(int i = 0; i < n - 1; i++)
    {
        Serial.print(historySpeed[i]);
        Serial.print(" ");
    }
    Serial.println(historySpeed[n - 1]);
#endif

    return historySpeed[n - 1];
}

double SpeedCalibrate()
{
    /*
        50 Meters
    ---------------- *1000000 = Meters per pulse*1000000
    number of pulses

    press calibration button, roll 50m
    in the moment that its done press calibration button again.

    so the program can associate number of pulses to the known value that is 50 meters
     */
    /*
     bool HIGHState = true;
     bool LOWState = false;
     bool LOOPState = true;
     int PulseCount = 0;

     while(LOOPState){
         bool PulseState = digitalRead(SensorIn);

         if(PulseState && HIGHState){
           PulseCount++;
           HIGHState = false;
           LOWState = true;
         }
         else if(!PulseState && LOWState){
           PulseCount++;
           HIGHState = true;
           LOWState = false;
         }

         if(digitalRead(SpeedCalibrateButton)) LOOPState=false;
     }*/

    detachInterrupt(sensorInterrupt);
    pulseCountSpeed = 0;
    unsigned long SpeedLedTimer = millis();

    attachInterrupt(sensorInterrupt, pulseCounterSpeed, CHANGE);

    while(!digitalRead(SpeedCalibrateButton))
    {   
        if(millis()-SpeedLedTimer>=SpeedLedBlink){
          SpeedLedState=!SpeedLedState;
          SpeedLedTimer = millis();
        } 
        digitalWrite(SpeedLed, SpeedLedState);
        Serial.println(pulseCountSpeed);
    }

    detachInterrupt(sensorInterrupt);
    MeterPP = (50.00 / pulseCountSpeed) * pow(10, 6); //need to store on eeprom;

    attachInterrupt(sensorInterrupt, PulseTime, CHANGE);


    return MeterPP;

}



bool MinSpeedTester(double Spd, int minSpd){

    if(Spd<minSpd){
      if(n>=N_MinSpd){
        n=0;
        LastState = false;
        return false;
      }
      //the function will return false only when 
      //the speed was teste at least N_MinSpd 
      //times and being smaller then the minSpd
      n++; 
    }
    else{
      n=0;
      LastState = true;
      return true;
    } 



    return LastState;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//INTERRUPTER ROUTINES

void pulseCounterSpeed()
{
    pulseCountSpeed++;
}

void PulseTime()
{

    PulseDurationSpeed = micros() - oldTimeSpeed;
    if(PulseDurationSpeed<0) PulseDurationSpeed*=-1;
    oldTimeSpeed = micros();
}






