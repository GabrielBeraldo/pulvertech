
/*Telemetry return: SpeedVal, FluxLPM, SetPointLPM, setPoint, FeedbackFlux, dutyCycle, ValveState */

/*
*	Atmega328 pins:
*
*	Speed Sensor 		D2
*	Flux Sensor 		D3
*	Session Valve 1 	D7 & D8
*	H brige 			D9 & D10
*
*	Callibration Flux  A2
*	Flux Led			6
*	Callibration Speed 	A3
*	Speed Led			5
*
*	Telemetry RX		A4
*	Telemetry TX		A5
*
*	SD_CS 				D4
*	SD_SCK				D13
*	SD_MOSI				D12
*	SD_MISO				D11
*
*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Configurations.h"
#include "Debug.h"
#include "pins_arduino.h"
#include "Speed.h"
#include "Flux.h"
#include "control.h"
#include "calculus.h"
#include "KalmanFilter.h"
#include "EEPROM_ANY.h"
#include "display.h"

bool acquisitionFirstRun=true;
bool acquisitionFirstStage = false;
unsigned long acquisitionStartTime=0;

float PulsesPerLiter = 450.0; //nominal value from fluxometer
float PulsesPerMeter = 20.32; //50000; //0.15*pow(10,6); //FURTHER INFO: done 1016 pulse conunt in first real test, so number was round for test purpouses
float LiterPerHa = 60.0;
float MachineWidth = 6.5;
byte CalibrationKey= 101; //Key that indicates that the values on eeprom was already set as default or calibrated

float Proportional	= 0.7;
float Integrative	= 0.3;
float Derivative	= 180;

bool ValveState = true;
bool SpeedSimulationState = false;
unsigned long lastUpdate = millis();

float SpeedVal = 0.0;
float FluxLPM = 0.0;
float SetPointLPM = 0.0;

String telemetryBuffer = "";

SimpleKalmanFilter KF_SetPoint 	= SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_Flux 		= SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_SpeedVal 	= SimpleKalmanFilter(s_EMEA, s_EEST, s_Q);

SoftwareSerial TelemetrySerial(A4, A5); // RX, TX

ControllerClass Controller;
SpeedClass Speed;
FluxClass Flux;
nextionDisplay Display;

void eepromWriteParameters(){

	eepromWrite(MeterCalibrationAdd, PulsesPerMeter);
	eepromWrite(MachineWidthAdd, MachineWidth);
	eepromWrite(LiterPerHaAdd, LiterPerHa);
	eepromWrite(PulsesPerLiterAdd, PulsesPerLiter);
	eepromWrite(CalibStateAdd, CalibrationKey);

	eepromWrite(ProportionalAdd, Proportional);
	eepromWrite(IntegrativeAdd, Integrative);
	eepromWrite(DerivativeAdd, Derivative);


	TelemetrySerial.println("Calibration stored");
	TelemetrySerial.println(EEPROM.read(0));
}

void eepromReadParameters(){

	eepromRead(MeterCalibrationAdd, PulsesPerMeter);
	eepromRead(MachineWidthAdd, MachineWidth);
	eepromRead(LiterPerHaAdd, LiterPerHa);
	eepromRead(PulsesPerLiterAdd, PulsesPerLiter);

	eepromRead(ProportionalAdd, Proportional);
	eepromRead(IntegrativeAdd, Integrative);
	eepromRead(DerivativeAdd, Derivative);

}

void storedParameters(){
	
	TelemetrySerial.print("PPM: ");
	TelemetrySerial.println(PulsesPerMeter);
	TelemetrySerial.print("PPL: ");
	TelemetrySerial.println(PulsesPerLiter);
	TelemetrySerial.print("MachineWidth: ");
	TelemetrySerial.println(MachineWidth);
	TelemetrySerial.print("LiterPerHa: ");
	TelemetrySerial.println(LiterPerHa);

	TelemetrySerial.print("Proportional: ");
	TelemetrySerial.println(Proportional);
	TelemetrySerial.print("Integrative: ");
	TelemetrySerial.println(Integrative);
	TelemetrySerial.print("Derivative: ");
	TelemetrySerial.println(Derivative);

}

void controlRoutine(){
	//=================================================
	//Get Variables values and calculate all needed to run the PIDControl function

	//=================================================
	//flux
	FluxLPM = Flux.update(PulsesPerLiter/100);
	if(FluxLPM>30) FluxLPM=30;
	if(FluxLPM<1) FluxLPM=0;
	FluxLPM = Flux.average(FluxLPM, 5);
	//=================================================
	
	//=================================================
	//speed
		
	if(Display.Simulation()){
		SpeedVal = Display.GetSimSpeed();
	}
	else{	
		SpeedVal = Speed.update(PulsesPerMeter);
		SpeedVal = Speed.average(SpeedVal, 5);
		SpeedVal = constrain(SpeedVal, 0, MaxSpeedVal(LiterPerHa, MachineWidth));
		SpeedVal = KF_SpeedVal.updateEstimate(SpeedVal);
	}
	//=================================================

	//=================================================
	//setpoint
	SetPointLPM = Calculate(LiterPerHa, SpeedVal, MachineWidth); 	//L/min, km/h, meters
	SetPointLPM = constrain(SetPointLPM, 0, 20);

	int setPoint = convert2SetPoint(SetPointLPM);              			//convert liter per min into setPoint, going from 0 to 1023(10bits)
	int FeedbackFlux = convert2SetPoint(FluxLPM);


	FeedbackFlux = KF_Flux.updateEstimate(FeedbackFlux);
	setPoint = KF_SetPoint.updateEstimate(setPoint);
	
	bool minSpd = Speed.testMin(SpeedVal, MinSpeed);
	int dutyCycle = 0;
	//=================================================
	
	//=================================================
	//Apply Flux Control Routines

	if(minSpd  && !ValveState && Display.Application() ) //tests if the speed is higher then min speed and the valve is closed, then open it
	{

		TelemetrySerial.println("Opening Valve");
		Controller.openSession();
		ValveState = true; //indicates that the valve is now open

	}
	else if ((!minSpd && ValveState) || (!Display.Application() && ValveState)) //tests if the speed is smaller then min speed and the valve is open, then close it
	{

		TelemetrySerial.println("Closing Valve");
		Controller.closeSession();
		Controller.releaseBridge();
		ValveState = false; //indicates that the valve is now closed
	}
	else if(minSpd && ValveState && Display.Application())
	{
		dutyCycle = Controller.controllerDuty();
		Controller.update(FeedbackFlux, setPoint);
	}


	#ifdef telemetry
		float Ts = ((millis()-lastUpdate))/1000.00;

		telemetryBuffer = String(SpeedVal) + ", " 
						+ String(FluxLPM) + ", " 
						+ String(SetPointLPM) + ", " 
						+ String(setPoint) + ", " 
						+ String(FeedbackFlux) + ", " 
						+ String(dutyCycle) + ", " 
						+ String(ValveState) + ", " 
						+ String(Ts);

		#if !SILENT_TELEMETRY
			TelemetrySerial.println(telemetryBuffer);
		#endif
	#endif

}

void updateDisplayValues(){

	int SpeedVal_toDisplay 		= SpeedVal*10;
	int FluxLPM_toDisplay		= FluxLPM*10;
	int SetPointLPM_toDisplay	= SetPointLPM*10;
	int controllerStability 	= Controller.stability()? 1 : 0;
	int simulationState 		= Display.Simulation()? 1 : 0;
	int displayApplicationState = Display.Application()? 1 : 0;

	Display.sendCmd("Main.speed.val=" 		+ String(SpeedVal_toDisplay));
	Display.sendCmd("Main.flux.val=" 		+ String(FluxLPM_toDisplay));
	Display.sendCmd("Main.setPoint.val=" 	+ String(SetPointLPM_toDisplay));
	Display.sendCmd("Main.stability.val=" 	+ String(controllerStability));
	Display.sendCmd("Main.contState.val=" 	+ String(displayApplicationState));
	Display.sendCmd("Main.simState.val=" 	+ String(simulationState));

}

void setup()
{

	TelemetrySerial.begin(BaudRate);
	TelemetrySerial.println("");

	//////////////////////////////////////////////////////////////////
	//Test if calibration was set as valid(first boot), in case it's not, store default values and set as valid values
	byte CalibStateRead = EEPROM.read(0);
	if(CalibStateRead != CalibrationKey) eepromWriteParameters();
	else TelemetrySerial.println("Ok EEPROM Key");
	////////////////////////////////////////////////////////////////
	
	eepromReadParameters();
	storedParameters();

	Speed.setup();
	Flux.setup();
	Controller.setup(Proportional, Integrative, Derivative);
	Display.setup(PulsesPerLiter, PulsesPerMeter, LiterPerHa, MachineWidth, Controller.proportional(), Controller.integrative(), Controller.derivative());


	TelemetrySerial.println("initialization done.");

	lastUpdate = millis();
}

void loop()
{

	String received = Display.updateInput();
	//if(received.length()>0) TelemetrySerial.println(received);
	
	if(Display.available()){

		PulsesPerLiter	= Display.GetPPL();
		PulsesPerMeter	= Display.GetPPM();
		LiterPerHa		= Display.GetLPHa();
		MachineWidth	= Display.GetmWidth();

		Proportional	= Display.GetProportional();
		Integrative 	= Display.GetIntegrative();
		Derivative		= Display.GetDerivative();

		Controller.proportional(Proportional); 
		Controller.integrative(integrative);
		Controller.derivative(derivative);

		eepromWriteParameters();	
		storedParameters();

		Display.usedData();	
	}
	
	//ensure precise Ts of control parameters
	if((millis() - lastUpdate)>=sampleTime){

		controlRoutine();
		updateDisplayValues();
		lastUpdate = millis();
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
}
