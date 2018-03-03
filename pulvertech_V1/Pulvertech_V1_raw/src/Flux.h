//#define debugFlux
//#define debugFluxAverage
//#include "Arduino.h"
#include "wiring_private.h"
#include "pins_arduino.h"


#define sensorInterruptFlux 0  // 0 = digital pin 2
#define SensorInFlux 2//A1
#define timeoutFlux 500
#define FluxCalibrateButton 5//A4
#define FluxLed 6//A3
#define FluxLedBlink 420
//////////////////////////////////////////////////////////////////////////////////////
//////initialization of vars and functions///////////////////////////////////////////

int flux = 0 ;
int h1, h2, h3;
float historyFlux[10];
double LiterPP = 0.1; //need to read from eeprom
float LastLPM = 0;
volatile int pulseCount;
volatile int PulseDuration = 0;
unsigned long oldTime = 0;
unsigned long MillisTimeout = 0;
bool FluxLedState=false;

void FluxSetup();
float ReadFlux(float);
int FluxAverage(int);
double FluxCalibrate();
void pulseCounter();
void FluxTime();

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void FluxSetup()
{
    pinMode(SensorInFlux, INPUT);
    pinMode(FluxCalibrateButton, INPUT);
    pinMode(FluxLed, OUTPUT);

    for(int i = 0; i < 50; i++) historyFlux[i] = 0;

    attachInterrupt(sensorInterruptFlux, FluxTime, CHANGE);
}

float ReadFlux(float LPP)
{

    //int LPM = 0;
    //int PulseTime=abs(pulseIn(SensorIn,HIGH,timeout));
    //if(PulseTime<0) PulseTime*=-1;
    /*
    float LPM = (LPP/PulseTime)*60.0;
    */

    float LPM = 0;
    if(PulseDuration > 0) LPM = (LPP / PulseDuration) * 60.0;


    if(LPM == LastLPM)
    {
        if(millis()- MillisTimeout >= timeoutFlux) LPM = PulseDuration = 0;

    }
    else
    {
        MillisTimeout = millis();
        LastLPM = LPM;
    }

#ifdef debugFlux
    Serial.print(PulseDuration);
    Serial.print(" ");
    Serial.println(LPM);
#endif

    if(LPM > 0 && LPM < 100) return(LPM);
    else return 0;
}

float FluxAverage(float flx, int n)
{

    float avrg = 0;
    if(flx <= 0) flx = 0;

    for(int i = 1; i < n; i++) historyFlux[i - 1] = historyFlux[i];
    historyFlux[n - 1] = flx;
    for(int i = 0; i < n; i++) avrg += historyFlux[i];
    historyFlux[n - 1] = avrg / n;

#ifdef debugFluxAverage
    for(int i = 0; i < n - 1; i++)
    {
        Serial.print(historyFlux[i]);
        Serial.print(" ");
    }
    Serial.println(historyFlux[n - 1]);
#endif

    return historyFlux[n - 1];
}

double FluxCalibrate()
{
    /*
        1 liter
    ---------------- *1000000 = liter per pulse*1000
    number of pulses

    press calibration button, fill a 20Liter bottle
    in the moment that its done press calibration button again.

    so the program can associate number of pulses to the known value that is 20 liters
     */
    /*
     bool HIGHState = true;
     bool LOWState = false;
     bool LOOPState = true;
     int PulseCount = 0;

     while(LOOPState){
         bool PulseState = digitalRead(SensorIn);

         if(PulseState == 1 && HIGHState){
           PulseCount++;
           HIGHState = false;
           LOWState = true;
         }
         else if(!PulseState == 0 && LOWState){
           PulseCount++;
           HIGHState = true;
           LOWState = false;
         }

         if(digitalRead(FluxCalibrateButton)) LOOPState=false;
     }
    */

    detachInterrupt(sensorInterruptFlux);
    pulseCount = 0;
    unsigned long FluxLedTimer = millis();

    attachInterrupt(sensorInterruptFlux, pulseCounter, CHANGE);

    while(!digitalRead(FluxCalibrateButton))
    { 
        if(millis()-FluxLedTimer>=FluxLedBlink){
          FluxLedState=!FluxLedState;
          FluxLedTimer = millis();
        } 
        digitalWrite(FluxLed, FluxLedState);
        Serial.println(pulseCount);
    }
    LiterPP = (1.00 / pulseCount) * pow(10, 6);

    detachInterrupt(sensorInterruptFlux);
    attachInterrupt(sensorInterruptFlux, FluxTime, CHANGE);
    return LiterPP;

}
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


void pulseCounter()
{

    pulseCount++;
}

void FluxTime()
{

    PulseDuration = micros() - oldTime;
    if(PulseDuration<0) PulseDuration*=-1;
    oldTime = micros();
}










