#include <Arduino.h>

#define sensorInterruptFlux 1  // 0 = digital pin 2
#define SensorInFlux 3//A1

/////////////////////////////////////
//interrupter functions ust be global
volatile int pulseCount;
unsigned long millisOld = 0;

void pulseCounter()
{
    pulseCount++;
}

/////////////////////////////////////////

class FluxClass{

    int flux = 0 ;
    float historyFlux[10];
  
  public:

    void setup();
    float update(float);
    float average(float, int);
};

void FluxClass::setup()
{
    pinMode(SensorInFlux, INPUT);

    for(int i = 0; i < 10; i++) historyFlux[i] = 0;

    attachInterrupt(sensorInterruptFlux, pulseCounter, CHANGE);

    millisOld = millis();
}

float FluxClass::update(float calibrationFactor)
{

    detachInterrupt(sensorInterruptFlux);

    int duration = millis()-millisOld;

    float flowRate = ((1000/duration)*pulseCount)/ calibrationFactor;

    millisOld=millis();
    pulseCount=0;

    attachInterrupt(sensorInterruptFlux, pulseCounter, CHANGE);

    return flowRate;
}

float FluxClass::average(float flx, int n)
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









