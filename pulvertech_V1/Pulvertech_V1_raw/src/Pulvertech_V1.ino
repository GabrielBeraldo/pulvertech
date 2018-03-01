//#define VarDebug
#define telemetry

/*Telemetry return: SpeedVal, FluxLPM, SetPointLPM, setPoint, FeedbackFlux, duttyCicle, ValveState */

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
*	SD_CS 				D4
*	SD_SCK				D13
*	SD_MOSI				D12
*	SD_MISO				D11
*
*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "pins_arduino.h"
#include "Speed.h"
#include "Flux.h"
#include "control.h"
#include "calculus.h"
#include "KalmanFilter.h"
#include "EEPROM_ANY.h"
#include "SDcard.h"
//#include "BoardAuth.h"


#define BaudRate 9600
#define MinSpeed 2.0
#define DelayValve 3000 //3 seconds for open/close the section valve
#define ButtonPressTime 1000
#define SpeedSimulationVal 5
#define SpeedSimulationButton A2 //5
#define SpeedSimulationLed 6//6

#define MeterCalibrationAdd 1
#define LiterCalibrationAdd 25
#define MachineWidthAdd 50
#define LiterPerHaAdd 75
#define CalibStateAdd 0

#define _EMEA 100
#define _EEST 100
#define _Q 0.1

#define s_EMEA 100
#define s_EEST 100
#define s_Q 0.4


double MetersPerPulse = 49212; //50000; //0.15*pow(10,6); //FURTHER INFO: done 1016 pulse conunt in first real test, so number was round for test purpouses
double LitersPerPulse = 1230;  //real calibration number Set as Default
int LiterPerHa = 60;
float MachineWidth = 6.0;
byte CalibrationKey=010; //Key that indicates that the values on eeprom was already set as default or calibrated

float SdData[5];
bool SdValid = false;
bool ValveState = true;
bool SpeedSimulationState = false;
unsigned long MillisOld = 0;

SimpleKalmanFilter KF_SetPoint = SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_Flux = SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_SpeedVal = SimpleKalmanFilter(s_EMEA, s_EEST, s_Q);

SoftwareSerial TelemetrySerial(A4, A5); // RX, TX

void setup()
{

	TelemetrySerial.begin(BaudRate);
	TelemetrySerial.println("");
		//////////////////////////////////////////////////////////////////
		//Test if calibration was set as valid(first boot), in case it's not, store default values and set as valid values
		byte CalibStateRead = EEPROM.read(0);
		if(CalibStateRead != CalibrationKey)
		{
			eepromWrite(MeterCalibrationAdd, MetersPerPulse);
			eepromWrite(LiterCalibrationAdd, LitersPerPulse);
			eepromWrite(MachineWidthAdd, MachineWidth);
			eepromWrite(LiterPerHaAdd, LiterPerHa);
			eepromWrite(CalibStateAdd, CalibrationKey);

			TelemetrySerial.println("CalibrationKey stored");
			TelemetrySerial.println(EEPROM.read(0));

		}
		else TelemetrySerial.println("Ok EEPROM Key");
		////////////////////////////////////////////////////////////////
	
	

	pinMode(SpeedSimulationButton, INPUT);
	pinMode(SpeedSimulationLed, OUTPUT);

	TelemetrySerial.println("Starting variables");
	
	eepromRead(MeterCalibrationAdd, MetersPerPulse);
	eepromRead(LiterCalibrationAdd, LitersPerPulse);
	eepromRead(MachineWidthAdd, MachineWidth);
	eepromRead(LiterPerHaAdd, LiterPerHa);
	
	TelemetrySerial.print("MPP: ");
	TelemetrySerial.println(MetersPerPulse);
	TelemetrySerial.print("LPP: ");
	TelemetrySerial.println(LitersPerPulse);
	TelemetrySerial.print("MachineWidth: ");
	TelemetrySerial.println(MachineWidth);
	TelemetrySerial.print("LiterPerHa: ");
	TelemetrySerial.println(LiterPerHa);


	SpeedSetup();
	FluxSetup();
	PIDSetup();

	TelemetrySerial.println("initialization done.");

}

void loop()
{

	//////////////////////////////////////////////////////////////////////////////////////////
	//Check for any user request /////////////////////////////////////////////////////////////


	if(digitalRead(SpeedSimulationButton))
	{
		MillisOld = millis();

		while(digitalRead(SpeedSimulationButton))
		{
			if((millis() - MillisOld) >= ButtonPressTime)
			{
				SpeedSimulationState = !SpeedSimulationState;
				digitalWrite(SpeedSimulationLed,SpeedSimulationState);

				TelemetrySerial.print("SpeedSimulation set as: ");
				TelemetrySerial.println(SpeedSimulationState);

				while(digitalRead(SpeedSimulationButton));
			}
		}
	}

	if(digitalRead(SpeedCalibrateButton))
	{
		MillisOld = millis();

		while(digitalRead(SpeedCalibrateButton))
		{
			if((millis() - MillisOld) >= ButtonPressTime)
			{

				TelemetrySerial.println("SpeedCalibrate Start");
				digitalWrite(SpeedLed, 1);
				while(digitalRead(SpeedCalibrateButton));

				MetersPerPulse = SpeedCalibrate();
				eepromWrite(MeterCalibrationAdd, MetersPerPulse);

				TelemetrySerial.println(MetersPerPulse);
				TelemetrySerial.println("SpeedCalibrate end");
				digitalWrite(SpeedLed, 0);
				MillisOld = millis();
				while(digitalRead(SpeedCalibrateButton));
			}
		}
	}
	/*
	if(digitalRead(FluxCalibrateButton))
	{
		MillisOld = millis();
		while(digitalRead(FluxCalibrateButton))
		{
			if((millis() - MillisOld) >= ButtonPressTime)
			{

				TelemetrySerial.println("FluxCalibrate Start");
				digitalWrite(FluxLed, 1);
				while(digitalRead(FluxCalibrateButton));

				LitersPerPulse = FluxCalibrate();
				eepromWrite(LiterCalibrationAdd, LitersPerPulse);

				TelemetrySerial.println(LitersPerPulse);
				TelemetrySerial.println("FluxCalibrate end");
				digitalWrite(FluxLed, 0);
				MillisOld = millis();
				while(digitalRead(FluxCalibrateButton));
			}
		}
	}*/

	//////////////////////////////////////////////////////////////////////////////////////////
	//Get Variables values and calculate all needed to run the PIDControl function////////////

	float FluxLPM = ReadFlux(LitersPerPulse);
	FluxLPM = FluxAverage(FluxLPM, 5);
	if(FluxLPM<1) FluxLPM=0;

	float SpeedVal = 0;

	if(SpeedSimulationState) SpeedVal = SpeedSimulationVal;
	else{
		SpeedVal=ReadSpeed(MetersPerPulse);
		SpeedVal = SpeedAverage(SpeedVal, 5);
		SpeedVal = constrain(SpeedVal, 0, MaxSpeedVal(LiterPerHa, MachineWidth));
		SpeedVal = KF_SpeedVal.updateEstimate(SpeedVal);
	}

	float SetPointLPM = Calculate(LiterPerHa, SpeedVal, MachineWidth); 	//L/min, km/h, meters
	SetPointLPM = constrain(SetPointLPM, 0, 20);

	float setPoint = convert2SetPoint(SetPointLPM);              			//convert liter per min into setPoint, going from 0 to 1023(10bits)
	float FeedbackFlux = convert2SetPoint(FluxLPM);


	FeedbackFlux = KF_Flux.updateEstimate(FeedbackFlux);
	setPoint = KF_SetPoint.updateEstimate(setPoint);

	//////////////////////////////////////////////////////////////////////////////////////////////////
	//Apply Flux Control Routines/////////////////////////////////////////////////////////////////////

	bool minSpd = MinSpeedTester(SpeedVal, MinSpeed);
	int duttyCicle = 0;

	if(minSpd  && !ValveState) //tests if the speed is higher then min speed and the valve is closed, then open it
	{

		TelemetrySerial.println("Opening Valve");
		OpenSession();
		ValveState = true; //indicates that the valve is now open

	}
	else if (!minSpd && ValveState) //tests if the speed is smaller then min speed and the valve is open, then close it
	{

		TelemetrySerial.println("Closing Valve");
		CloseSession();
		ValveState = false; //indicates that the valve is now closed
	}
	else if(minSpd && ValveState)
	{
		duttyCicle = PIDControl(FeedbackFlux, setPoint);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////



	#ifdef telemetry

	TelemetrySerial.print(SpeedVal);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(FluxLPM);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(SetPointLPM);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(setPoint);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(FeedbackFlux);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(duttyCicle);

	TelemetrySerial.print(F(", "));
	TelemetrySerial.println(ValveState);

	#endif



}
