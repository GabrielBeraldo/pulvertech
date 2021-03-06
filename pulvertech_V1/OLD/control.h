//#define debugPID //for debug uncomment this line

//#include "Arduino.h"
//////////////////////////////////////////////////////////////////////////////////////
/////Configuration of PID/////////////////////////////////////////////////////////////
#define P_FRACTION 0.8//0.8         //0.0 - 10.0 (0.3)
#define I_FRACTION 0.08//0.08         //0.0 - 10.0 (0.3)
#define D_FRACTION 6.0//6.0         //0.0 - 10.0 (4.0)

#define V_WINDOW 5            //10 - 1000 (25)
#define MIN_DUTYCYCLE 1       //0 - 255 (25)
#define MAX_DUTYCYCLE 255      //0 - 255 (255)
#define SOFT_START 0.1         //0.00 - 1.00 (0.30) 1.00 = OFF
#define EMERGENCY_SHUTDOWN 0   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE 10 //Prevent H bridge from shoot through whenever the direction pin is changed
#define LOOP_DELAY 0

#define VALVE_PIN 10
#define VALVE_PIN2 9
#define VALVE_SESSION_PIN 8
#define VALVE_SESSION_PIN2 7

//////////////////////////////////////////////////////////////////////////////////////
//////initialization of vars and functions///////////////////////////////////////////
int ADC_SetPoint = 0;
int ADC_SetPointOld = 0;
int FluxIn = 0;
int FluxInOld = 0;
int dutyCycle = 0; // 10 - 255
int ADCdiff = 0;
int timeDiff = 0;

void PIDSetup();
int PIDControl(int);
void ReleaseBridge();
void Open(int);
void Close(int);
void OpenSession();
void CloseSession();
void ReleaseSession();

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void PIDSetup()
{
    pinMode(VALVE_PIN, OUTPUT);
    pinMode(VALVE_PIN2, OUTPUT);
    pinMode(VALVE_SESSION_PIN, OUTPUT);
    pinMode(VALVE_SESSION_PIN2, OUTPUT);

}


int PIDControl(int FluxIn, int ADC_SetPoint)  // reads the flux sensor (between 0 and 1023)
{

    ADCdiff = ADC_SetPoint - FluxIn;

    //PID implenetatiom
    dutyCycle = abs(ADCdiff) * P_FRACTION;
    dutyCycle += timeDiff * I_FRACTION;
    dutyCycle += abs(ADC_SetPointOld - ADC_SetPoint) * D_FRACTION;
    //end of calculus of error on PID



    if(SOFT_START * timeDiff < 1)
    {
        dutyCycle = dutyCycle * (SOFT_START * timeDiff);
    }

    timeDiff++;

    if(dutyCycle < MIN_DUTYCYCLE && dutyCycle > 0)
    {
        dutyCycle = MIN_DUTYCYCLE;
    }

    if(dutyCycle > MAX_DUTYCYCLE)
    {
        dutyCycle = MAX_DUTYCYCLE;
    }

    if(dutyCycle < 0)
    {
        dutyCycle = 0;
    }


    if(abs(ADCdiff) < V_WINDOW)
    {
        dutyCycle = 0;
        timeDiff = 0;
    }

    if(abs(FluxInOld - FluxIn) < EMERGENCY_SHUTDOWN && dutyCycle == MAX_DUTYCYCLE && timeDiff > 50)
    {
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        ReleaseBridge();
        delay(1000);
        timeDiff = 0;
    }
    else
    {
        if(ADCdiff > 0)
        {
            ReleaseBridge();
            delayMicroseconds(SHOOT_THROUGH_PAUSE);
            Open(dutyCycle);
        }
        if(ADCdiff < 0)
        {
            ReleaseBridge();
            delayMicroseconds(SHOOT_THROUGH_PAUSE);
            Close(dutyCycle);
        }
    }



    ADC_SetPointOld = ADC_SetPoint;
    FluxInOld = FluxIn;

#ifdef debugPID
    Serial.print(FluxIn);
    Serial.print(" ");
    Serial.print(ADC_SetPoint);
    Serial.print(" ");
    Serial.println(dutyCycle);
#endif
    delay(LOOP_DELAY);
    return dutyCycle;
}


void Open(int dutty)
{
    analogWrite(VALVE_PIN, 0 );
    analogWrite(VALVE_PIN2, dutty );
}

void Close(int dutty)
{
    analogWrite(VALVE_PIN, dutty );
    analogWrite(VALVE_PIN2, 0 );
}

void ReleaseBridge()
{
    analogWrite(VALVE_PIN, 0);
    analogWrite(VALVE_PIN2, 0);
}


void OpenSession()
{
    digitalWrite(VALVE_SESSION_PIN, 1);
    digitalWrite(VALVE_SESSION_PIN2, 0);
}

void CloseSession()
{
    digitalWrite(VALVE_SESSION_PIN, 0);
    digitalWrite(VALVE_SESSION_PIN2, 1);

}

void ReleaseSession()
{
    digitalWrite(VALVE_SESSION_PIN, 0);
    digitalWrite(VALVE_SESSION_PIN2, 0);

}