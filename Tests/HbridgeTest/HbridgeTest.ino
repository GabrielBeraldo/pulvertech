
#define VALVE_PIN 9  
#define VALVE_PIN2 10

#define SECTION 8

#define delayVal 5

void Open(int);
void Close(int);
void ReleaseBridge(int);



void setup(){
	Serial.begin(9600);
	pinMode(SECTION, OUTPUT);
	pinMode(VALVE_PIN, OUTPUT);
	pinMode(VALVE_PIN2, OUTPUT);
  Serial.println("start");

  //TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz
}

void loop(){

	for(int i=0; i<255; i++){
		Open(i);
		delay(delayVal);
	}
	for(int i=255; i>0; i--){
		Open(i);
		delay(delayVal);
	}
	Serial.println("Close");
	ReleaseBridge();
	delay(delayVal+1000);

	for(int i=0; i<255; i++){
		Close(i);
		delay(delayVal);
	}
	for(int i=255; i>0; i--){
		Close(i);
		delay(delayVal);
	}
	ReleaseBridge();
	delay(delayVal+1000);
	Serial.println("Open");

}


void Open(int dutty){
  analogWrite(VALVE_PIN, 0 );
  analogWrite(VALVE_PIN2, dutty );
  digitalWrite(SECTION, 1);
}

void Close(int dutty){
  analogWrite(VALVE_PIN, dutty );
  analogWrite(VALVE_PIN2, 0 );
  digitalWrite(SECTION, 0);
}

void ReleaseBridge(){
   analogWrite(VALVE_PIN, 0);
   analogWrite(VALVE_PIN2, 0);
}
