
#include <arduino.h>

#define sensorInterruptSpeed 0  // 1 = digital pin 3
#define SensorInSpeed 2 //A0
#define timeoutSpeed 500
#define SpeedCalibrateButton A3
#define SpeedLed 5
#define SpeedLedBlink 420
#define N_MinSpd 20
#define MetersForCalibrate5 50

/////////////////////////////////////
//interrupter functions must be global

volatile long pulseCountSpeed;
volatile long PulseDurationSpeed = 0;
unsigned long oldTimeSpeed = 0;
unsigned long MillisTimeoutSpeed = 0;

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
/////////////////////////////////////

class SpeedClass{

    float historySpeed[10];
    float PulsesPerMeter = 0; //need to read from eeprom
    float LastKMPH = 0;
    bool SpeedLedState=false;
    int n=0;
    bool LastState = false;

 public:

    void setup();
    float update(float PPM);
    float average(float, int);
    float calibrate();
    bool testMin(float, int);
};


void SpeedClass::setup()
{
    pinMode(SensorInSpeed, INPUT);
    pinMode(SpeedCalibrateButton, INPUT);
    pinMode(SpeedLed, OUTPUT);

    for(int i = 0; i < 10; i++) historySpeed[i] = 0;

    attachInterrupt(sensorInterruptSpeed, PulseTime, CHANGE);
}

float SpeedClass::update(float PPM)
{

    float KMPH = 0;
    if(PulseDurationSpeed > 0)
        KMPH = (1 / (PulseDurationSpeed * PPM)) * 3.6; //KMPH = ((MPP) / PulseDurationSpeed) * 3.6;



    if(KMPH == LastKMPH)
    {
        if( millis() - MillisTimeoutSpeed >= timeoutSpeed) KMPH = PulseDurationSpeed = 0;
    }
    else
    {
        MillisTimeoutSpeed = millis();
        LastKMPH = KMPH;
    }


    KMPH=constrain(KMPH,0,60);
    return(KMPH);
}

float SpeedClass::average(float Spd, int n)
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

float SpeedClass::calibrate()
{
    /*
    
    number of pulses 
    ---------------- = Pulses Per Meter
        50 Meters 

    press calibration button, roll 50m
    in the moment that its done press calibration button again.

    so the program can associate number of pulses to the known value that is 50 meters
     */

    detachInterrupt(sensorInterruptSpeed);
    pulseCountSpeed = 0;
    unsigned long SpeedLedTimer = millis();

    attachInterrupt(sensorInterruptSpeed, pulseCounterSpeed, CHANGE);

    while(!digitalRead(SpeedCalibrateButton))
    {
        if(millis()-SpeedLedTimer>=SpeedLedBlink){
          SpeedLedState=!SpeedLedState;
          SpeedLedTimer = millis();
        }
        digitalWrite(SpeedLed, SpeedLedState);
        Serial.println(pulseCountSpeed);
    }

    detachInterrupt(sensorInterruptSpeed);
    PulsesPerMeter = (pulseCountSpeed/MetersForCalibrate5); //need to store on eeprom;

    attachInterrupt(sensorInterruptSpeed, PulseTime, CHANGE);


    return PulsesPerMeter;

}

bool SpeedClass::testMin(float Spd, int minSpd){

    if(Spd<minSpd){
      if(n>=N_MinSpd){
        n=0;
        LastState = false;
        return false;
      }
      //the function will return false only when
      //the speed was tested at least N_MinSpd
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
