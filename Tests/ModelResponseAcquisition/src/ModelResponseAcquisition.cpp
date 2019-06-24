#include <Arduino.h>
#include "Flux.h"
#include "KalmanFilter.h"
#include "control.h"

//#define random
//#define openLoop

#define potPin 			A0

#define _EMEA 			50
#define _EEST 			50
#define _Q 				0.1

#define pulsesPerLiter 	450

#define minDuration 	100
#define maxDuration 	3000

#define maxSetPoint		15
#define minSetPoint		4

#define frequency 		90//Hz


#define Kp 		0.7
#define Ki		0.1
#define Kd 		187


//========================================================================
int setPoint 				=0;
int duration				=1000; //time for the system to stabilize 
unsigned long lastUpdate 	=millis();
unsigned long loopTime	 	=millis();
float filteredFlux 			=0;

bool direction 				=true;
int setPoint_ADC			=0;
float setPointVolts			=0;

float gains[]={Kp, Ki, Kd}; //P I D

//========================================================================
FluxClass Flux;
ControllerClass Controller;
SimpleKalmanFilter KF_Flux = SimpleKalmanFilter(_EMEA, _EEST, _Q);
//========================================================================

//========================================================================
void valuesUpdate(){
	#ifdef random
		setPoint = random(minSetPoint, maxSetPoint);
		duration = random(minDuration,maxDuration);
	#else
		#ifndef openLoop
			setPoint = map(analogRead(potPin),0,1023,minSetPoint,maxSetPoint);
		#else
			setPoint = map(analogRead(potPin),0,1023,-255,255);
			if(setPoint<0) direction = false;
			else direction = true;
			setPoint_ADC=abs(setPoint);
			setPointVolts=map(setPoint,-255,255,-500,500);
			setPointVolts/=100;
		#endif	
	#endif
}


//========================================================================
void updateSerial(){
	
	float Ts = ((millis()-loopTime));

	Serial.print(filteredFlux);
	Serial.print(",");
	#ifndef openLoop
		Serial.print(setPoint);
	#else
		Serial.print(setPointVolts);
	#endif
	Serial.print(",");
	Serial.print(Controller.controllerDuty());
  	Serial.print(",");
  	Serial.print(Controller.stability());
	Serial.print(",");
	Serial.print(Ts);
	Serial.println("");

}

//========================================================================
void syncLoopTime(){delay((1000/frequency)-(millis()-loopTime)); }
//ensure the acquisition frequency 

//========================================================================
void setup(){

	Serial.begin(9600);

	Flux.setup();
	Controller.setup();
	loopTime = millis();


	Controller.proporcional(gains[0]);
	Controller.integrative(gains[1]);
	Controller.derivative(gains[2]);
}

//========================================================================
void loop(){

	float fluxVal=Flux.update(pulsesPerLiter/100);
	filteredFlux = KF_Flux.updateEstimate(fluxVal);

	#ifdef random	
		if(millis()-lastUpdate>=duration){
			valuesUpdate();
			lastUpdate = millis();
		}
	#else
		valuesUpdate();
	#endif	


	int processValue_8 = map(filteredFlux,minSetPoint,maxSetPoint,0,255);
	int setPoint_8 = map(setPoint,minSetPoint,maxSetPoint,0,255);

	#ifndef openLoop
		Controller.update(processValue_8,setPoint_8);
	#else
		if(direction) Controller.open(setPoint_ADC);
		else Controller.close(setPoint_ADC);
	#endif
	
	syncLoopTime();
	updateSerial();
	loopTime = millis();

}
