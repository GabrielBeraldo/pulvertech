
#include <arduino.h>

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

#define StabilityCriteriaTime 5000
#define stabilityDeadband 20

class ControllerClass{

    float P_FRACTION=0; //#define P_FRACTION 2//0.8         //0.0 - 10.0 (0.3)
    float I_FRACTION=0;//#define I_FRACTION 0.08//0.08         //0.0 - 10.0 (0.3)
    float D_FRACTION=0;//#define D_FRACTION 6//6.0         //0.0 - 10.0 (4.0)

    int ADC_SetPoint = 0;
    int ADC_SetPointOld = 0;
    int FluxIn = 0;
    int FluxInOld = 0;
    int dutyCycle = 0; // 10 - 255
    int ADCdiff = 0;
    int timeDiff = 0;

    bool controllerStability = false;
    bool Stability = false;
    unsigned long StabilityTime=0;

    public:

    void setup();
    void update(int, int);
    void releaseBridge();
    void open(int);
    void close(int);
    void openSession();
    void closeSession();
    int controllerDuty(){return dutyCycle;};

    bool stability(){return controllerStability;};

    float proporcional(){return P_FRACTION;};
    void proporcional(float proporcional){P_FRACTION=proporcional;};

    float integrative(){return I_FRACTION;};
    void integrative(float integral){I_FRACTION=integral;};

    float derivative(){return D_FRACTION;};
    void derivative(float derivate){D_FRACTION=derivate;};
    

};

void ControllerClass::setup()
{
    pinMode(VALVE_PIN, OUTPUT);
    pinMode(VALVE_PIN2, OUTPUT);
    pinMode(VALVE_SESSION_PIN, OUTPUT);
}

void ControllerClass::update(int FluxIn, int ADC_SetPoint)  // reads the flux sensor (between 0 and 1023)
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
        Stability=true;
    }

    if(abs(FluxInOld - FluxIn) < EMERGENCY_SHUTDOWN && dutyCycle == MAX_DUTYCYCLE && timeDiff > 50)
    {
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        releaseBridge();
        delay(1000);
        timeDiff = 0;
    }
    else
    {   
        Stability=false;

        if(ADCdiff > 0)
        {
            releaseBridge();
            delayMicroseconds(SHOOT_THROUGH_PAUSE);
            open(dutyCycle);
        }
        if(ADCdiff < 0)
        {
            releaseBridge();
            delayMicroseconds(SHOOT_THROUGH_PAUSE);
            close(dutyCycle);
        }
    }

    /////////////////////////////////
    //stability timeout
    if(abs(ADCdiff) < stabilityDeadband){
        if((millis()-StabilityTime)>=StabilityCriteriaTime) 
        controllerStability=true;
    }
    else{
        controllerStability=false;
        StabilityTime=millis();
    }
    ///////////////////////////////////

    ADC_SetPointOld = ADC_SetPoint;
    FluxInOld = FluxIn;
    delay(LOOP_DELAY);
}

void ControllerClass::releaseBridge()
{
    analogWrite(VALVE_PIN, 0);
    analogWrite(VALVE_PIN2, 0);
}

void ControllerClass::open(int dutty)
{
    analogWrite(VALVE_PIN, 0 );
    analogWrite(VALVE_PIN2, dutty );
}

void ControllerClass::close(int dutty)
{
    analogWrite(VALVE_PIN, dutty );
    analogWrite(VALVE_PIN2, 0 );
}

void ControllerClass::openSession()
{
    digitalWrite(VALVE_SESSION_PIN, 1);
}

void ControllerClass::closeSession()
{
    digitalWrite(VALVE_SESSION_PIN, 0);

}


/*
//////////////////////////////////////////////////////////////////////////////////////
/////Configuration of PID/////////////////////////////////////////////////////////////
float P_FRACTION=2; //#define P_FRACTION 2//0.8         //0.0 - 10.0 (0.3)
float I_FRACTION=0;//#define I_FRACTION 0.08//0.08         //0.0 - 10.0 (0.3)
float D_FRACTION=0;//#define D_FRACTION 6//6.0         //0.0 - 10.0 (4.0)

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
void PID_Update_parameters(float, float, float);

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void PIDSetup()
{
    pinMode(VALVE_PIN, OUTPUT);
    pinMode(VALVE_PIN2, OUTPUT);
    pinMode(VALVE_SESSION_PIN, OUTPUT);

    Open(255);
    delay(5000);
    ReleaseBridge();
}

void PID_Update_parameters(float p, float i, float d){
    P_FRACTION = p;
    I_FRACTION = i;
    D_FRACTION = d;
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
}

void CloseSession()
{
    digitalWrite(VALVE_SESSION_PIN, 0);

}
*/