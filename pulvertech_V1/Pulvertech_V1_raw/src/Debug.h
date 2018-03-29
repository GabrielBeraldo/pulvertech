#include <Arduino.h>

class SerialCommands {
    int kmh = 0;
    int ppl = 1000;

    int p=0;
    int i=0;
    int d=0;

    int a=0;
    bool acquisitionState=false;

  public:
    void begin(int baud) {Serial.begin(baud);};
    void update();

    int Speed(){ return kmh; };
    int PulsesPerLiter(){return ppl;};

    float proporcional(){update(); return p;};
    float integrative(){update(); return i;};
    float derivative(){update(); return d;};
    
    float acquisition(){return acquisitionState;};
    void stopacquisition();

};

void SerialCommands::update() {


  if (Serial.available() > 0) {
    
    char peek = Serial.peek();

    switch(peek){
      case 's': 
        Serial.read();
        kmh = Serial.parseInt();
      break;

      case 'l':
        Serial.read();
        ppl = Serial.parseInt();
      break;
    
      case 'p':
        Serial.read();
        p= Serial.parseInt();
      break;

      case 'i':
        Serial.read();
        i = Serial.parseInt();
      break;

      case 'd':
        Serial.read();
        d = Serial.parseInt();
      break;

      case 'a':
      	Serial.read();
      	a = Serial.parseInt();
      	break;

    }
   
    while(Serial.available() > 0){Serial.read();}
  }


  if(a>0) acquisitionState=true;
  else acquisitionState=false;

};

void SerialCommands::stopacquisition(){
	acquisitionState=false;
	a=0;
}