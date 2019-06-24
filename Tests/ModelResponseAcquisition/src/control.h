
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
    float I_FRACTION=0; //#define I_FRACTION 0.08//0.08         //0.0 - 10.0 (0.3)
    float D_FRACTION=0; //#define D_FRACTION 6//6.0         //0.0 - 10.0 (4.0)

    int ADC_SetPoint = 0;
    int ADC_SetPointOld = 0;
    int ADC_processValue = 0;
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

void ControllerClass::update(int ADC_processValue, int ADC_SetPoint)  // reads the flux sensor (between 0 and 1023)
{

    ADCdiff = ADC_SetPoint - ADC_processValue;

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

    if(abs(FluxInOld - ADC_processValue) < EMERGENCY_SHUTDOWN && dutyCycle == MAX_DUTYCYCLE && timeDiff > 50)
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
    FluxInOld = ADC_processValue;
    delay(LOOP_DELAY);
}

void ControllerClass::releaseBridge()
{
    analogWrite(VALVE_PIN, 0);
    analogWrite(VALVE_PIN2, 0);
}

void ControllerClass::open(int duty)
{
    analogWrite(VALVE_PIN, 0 );
    analogWrite(VALVE_PIN2, duty );
    dutyCycle = duty;
}

void ControllerClass::close(int duty)
{
    analogWrite(VALVE_PIN, duty );
    analogWrite(VALVE_PIN2, 0 );
    dutyCycle = duty;
}

void ControllerClass::openSession()
{
    digitalWrite(VALVE_SESSION_PIN, 1);
}

void ControllerClass::closeSession()
{
    digitalWrite(VALVE_SESSION_PIN, 0);

}
