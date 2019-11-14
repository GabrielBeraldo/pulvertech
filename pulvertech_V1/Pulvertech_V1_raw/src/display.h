#include "Nextion.h"

float PPL, PPM, LPHa, mWidth, proportional, integrative, derivative, simulationSpeed;
bool updateDisplay = false;
bool displaySimulationMode = false;
bool applicationState = false;

NexVariable displayVarPPM     = NexVariable(1, 2,"Main.PPM");
NexVariable displayVarPPL     = NexVariable(1, 3,"Main.PPL");
NexVariable displayVarmWidth  = NexVariable(1, 4,"Main.mWidth");
NexVariable displayVarLPHa    = NexVariable(1, 5,"Main.LPHa");

NexVariable displayVarP    = NexVariable(1, 23,"Main.P");
NexVariable displayVarI    = NexVariable(1, 24,"Main.I");
NexVariable displayVarD    = NexVariable(1, 25,"Main.D");

NexVariable displaySimSpeed   = NexVariable(3, 16,"Debug.simSpeed");


void sendDisplayCmd(String cmd){
    Serial.print(cmd);   //Send instruction to clear the waveform
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
}

void sendParameters(){
    
    //adequate to float format of display
    int PPL_toDisplay       = PPL*10;  
    int PPM_toDisplay       = PPM*10;  
    int LPHa_toDisplay      = LPHa*10;  
    int mWidth_toDisplay    = mWidth*10;

    int proportional_toDisplay   = proportional*10;
    int integrative_toDisplay   = integrative*10;
    int derivative_toDisplay    = derivative*10;

    //send informations to dispaly
    sendDisplayCmd("Main.PPL.val=" +     String(PPL_toDisplay));
    sendDisplayCmd("Main.PPM.val=" +     String(PPM_toDisplay));
    sendDisplayCmd("Main.LPHa.val=" +    String(LPHa_toDisplay));
    sendDisplayCmd("Main.mWidth.val=" +  String(mWidth_toDisplay));
    
    sendDisplayCmd("Main.P.val=" +  String(proportional_toDisplay));
    sendDisplayCmd("Main.I.val=" +  String(integrative_toDisplay));
    sendDisplayCmd("Main.D.val=" +  String(derivative_toDisplay));

    sendDisplayCmd("bkcmd=0");
    sendDisplayCmd("page Main"); //goto mainpage
}

void storeDataCallback(){
    
    uint32_t value = 0;

    displayVarPPL.getValue(&value);
    PPL = value/10.0;

    delay(displayDelay);
    displayVarPPM.getValue(&value);
    PPM = value/10.0;

    delay(displayDelay);
    displayVarLPHa.getValue(&value);
    LPHa = value/10.0;

    delay(displayDelay);
    displayVarmWidth.getValue(&value);
    mWidth = value/10.0;

    delay(displayDelay);
    displayVarP.getValue(&value);
    proportional = value/10.0;

    delay(displayDelay);
    displayVarI.getValue(&value);
    integrative = value/10.0;

    delay(displayDelay);
    displayVarD.getValue(&value);
    derivative = value/10.0;

    updateDisplay = true; 
}

void SimulationRoutineCallback(){
    
    uint32_t value = 0;

    delay(displayDelay);
    displaySimSpeed.getValue(&value);
    simulationSpeed = value/10.0;

}

//=================================================================================
class nextionDisplay{


    public:
        void setup(float, float, float, float, float, float, float);
        bool available()    {return updateDisplay;};
        bool Simulation()   {return displaySimulationMode;};
        bool Application()  {return applicationState;};
        void usedData()     {updateDisplay = false;};
        
        float GetPPL()      {return PPL;};
        float GetPPM()      {return PPM;};
        float GetLPHa()     {return LPHa;};
        float GetmWidth()   {return mWidth;};
        float GetSimSpeed() {return simulationSpeed;};

        float GetProportional() {return proportional;}; 
        float GetIntegrative() {return integrative;};
        float GetDerivative() {return derivative;};

        void sendCmd(String data){sendDisplayCmd(data);};

        String updateInput();

};

void nextionDisplay::setup(float PPLval, float PPMval, float LPHaval, float mWidthval, float proportionalVal, float integrativeVal, float derivativeVal){
    Serial.begin(displayBaudrate);
    nexInit();

    PPL = PPLval;
    PPM = PPMval;
    LPHa = LPHaval;
    mWidth = mWidthval;
    proportional = proportionalVal; 
    integrative = integrativeVal; 
    derivative = derivativeVal;

    sendParameters();

}

String nextionDisplay::updateInput(){
    
    String Received ="";

    if(Serial.available()>0){ 
        while(Serial.available()>0){
            Received += Serial.readString();
            
        }
        

        switch(Received[0]){
            case 'S':
                storeDataCallback();
                break;
            
            case 'P':
                sendParameters();
                break;
        
            case 'D':
                displaySimulationMode = (Received[1]>0)? true : false; 
                SimulationRoutineCallback();
                break;

            case 'R':
                applicationState = (Received[1]>0)? true : false;


        }
        
      
    }

    return Received;

}