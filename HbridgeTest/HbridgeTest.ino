
#define VALVE_PIN 9  
#define VALVE_PIN2 10

void Open(int);
void Close(int);
void ReleaseBridge(int);


void setup(){
	Serial.begin(9600);
	pinMode(VALVE_PIN, OUTPUT);
	pinMode(VALVE_PIN2, OUTPUT);
}

void loop(){

	for(int i=0; i<255; i++){
		Open(i);
		delay(10);
	}
	for(int i=255; i>0; i--){
		Open(i);
		delay(10);
	}
	Serial.println("Close");
	ReleaseBridge();
	delay(10);

	for(int i=0; i<255; i++){
		Close(i);
		delay(10);
	}
	for(int i=255; i>0; i--){
		Close(i);
		delay(10);
	}
	ReleaseBridge();
	delay(10);
	Serial.println("Open");

}


void Open(int dutty){
  analogWrite(VALVE_PIN, 0 );
  analogWrite(VALVE_PIN2, dutty );
}

void Close(int dutty){
  analogWrite(VALVE_PIN, dutty );
  analogWrite(VALVE_PIN2, 0 );
}

void ReleaseBridge(){
   analogWrite(VALVE_PIN, 0);
   analogWrite(VALVE_PIN2, 0);
}
