#define debug 

#define Valve_Out 12
#define Sense_In 2
#define Threshold 0
#define timeoutpulse 2000

bool state=false; //true= open //false = closed
float historySpeed[10];
volatile long PulseDurationSpeed = 0;
unsigned long LastPDS=0;
unsigned long oldTimeSpeed = 0;
unsigned long MillisTimeoutSpeed = 0;

void PulseDuration();

void setup() {
  
  #ifdef debug
    Serial.begin(9600);
    Serial.println("Start");
  #endif
  
  pinMode(Valve_Out,OUTPUT);
  pinMode(Sense_In, INPUT);

  digitalWrite(Valve_Out, 0);
  attachInterrupt(digitalPinToInterrupt(Sense_In), PulseDuration, CHANGE);

  
}

void PulseDuration()
{

    PulseDurationSpeed = (micros() - oldTimeSpeed);
    
    if(PulseDurationSpeed<0) PulseDurationSpeed*=-1;
    oldTimeSpeed = micros();
}

void loop() {

  long pulse = PulseAverage(PulseDurationSpeed,10);

   if(PulseDurationSpeed == LastPDS)
    {
        if( millis() - MillisTimeoutSpeed >= timeoutpulse) pulse = PulseDurationSpeed = 0;
        for(int i=0;i<10;i++)historySpeed[i]=0;
        delay(100);
    }
    else
    {
        MillisTimeoutSpeed = millis();
        LastPDS=PulseDurationSpeed;
    }


  if(pulse>Threshold && !state){
    state = true;
    digitalWrite(Valve_Out, state);
  }
  else if(pulse<=Threshold && state){
    state = false;
    digitalWrite(Valve_Out,state);
    //delay(2000);

  }
 
#ifdef debug
 
 Serial.print(pulse);
 Serial.print(", ");
 Serial.print(PulseDurationSpeed);
 Serial.print(", ");
 Serial.print(millis() - MillisTimeoutSpeed);
 Serial.print(", ");
 Serial.println(state);

#endif
}



float PulseAverage(float pls, int n)
{

    float avrg = 0;
    if(pls <= 0) pls = 0;

    for(int i = 1; i < n; i++) historySpeed[i - 1] = historySpeed[i];
    historySpeed[n - 1] = pls;
    for(int i = 0; i < n; i++) avrg += historySpeed[i];
    historySpeed[n - 1] = avrg / n;

    return historySpeed[n - 1];
}

