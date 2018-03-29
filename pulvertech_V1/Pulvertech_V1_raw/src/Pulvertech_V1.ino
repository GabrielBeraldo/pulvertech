//#define VarDebug

#define telemetry
#define SerialCom

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
*	SD_CS 				D4
*	SD_SCK				D13
*	SD_MOSI				D12
*	SD_MISO				D11
*
*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Debug.h"
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
#define PulsesPerLiterAdd 25
//#define LiterCalibrationAdd 25
#define MachineWidthAdd 50
#define LiterPerHaAdd 75
#define CalibStateAdd 0

#define _EMEA 100
#define _EEST 100
#define _Q 0.1

#define s_EMEA 100
#define s_EEST 100
#define s_Q 0.5

#define loweracquisitionSetpoint 250
#define upperacquisitionSetpoint 500

bool acquisitionFirstRun=true;
bool acquisitionFirstStage = false;
unsigned long acquisitionStartTime=0;
//LitersPerPulse = (1.00 / PulsesPerLiter) * pow(10, 6);

int PulsesPerLiter = 2000; //813 aproximate standard for calbration number
float PulsesPerMeter = 20.32; //50000; //0.15*pow(10,6); //FURTHER INFO: done 1016 pulse conunt in first real test, so number was round for test purpouses
double LitersPerPulse = (1.00 / PulsesPerLiter) * pow(10, 6); // 1230;  //real calibration number Set as Default
int LiterPerHa = 60;
float MachineWidth = 6.0;
byte CalibrationKey=101; //Key that indicates that the values on eeprom was already set as default or calibrated

float SdData[5];
bool SdValid = false;
bool ValveState = true;
bool SpeedSimulationState = false;
unsigned long MillisOld = 0;

SimpleKalmanFilter KF_SetPoint = SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_Flux = SimpleKalmanFilter(_EMEA, _EEST, _Q);
SimpleKalmanFilter KF_SpeedVal = SimpleKalmanFilter(s_EMEA, s_EEST, s_Q);

SoftwareSerial TelemetrySerial(A4, A5); // RX, TX
SerialCommands Scom;
ControllerClass Controller;
SpeedClass Speed;
FluxClass Flux;

void setup()
{

	#ifdef SerialCom
		Scom.begin(9600);
	#endif

	TelemetrySerial.begin(BaudRate);
	TelemetrySerial.println("");

		//////////////////////////////////////////////////////////////////
		//Test if calibration was set as valid(first boot), in case it's not, store default values and set as valid values
		byte CalibStateRead = EEPROM.read(0);
		if(CalibStateRead != CalibrationKey)
		{
			eepromWrite(MeterCalibrationAdd, PulsesPerMeter);
			//eepromWrite(LiterCalibrationAdd, LitersPerPulse);
			eepromWrite(MachineWidthAdd, MachineWidth);
			eepromWrite(LiterPerHaAdd, LiterPerHa);
			eepromWrite(PulsesPerLiterAdd, PulsesPerLiter);
			eepromWrite(CalibStateAdd, CalibrationKey);

			TelemetrySerial.println("CalibrationKey stored");
			TelemetrySerial.println(EEPROM.read(0));

		}
		else TelemetrySerial.println("Ok EEPROM Key");
		////////////////////////////////////////////////////////////////
	
	

	pinMode(SpeedSimulationButton, INPUT);
	pinMode(SpeedSimulationLed, OUTPUT);

	TelemetrySerial.println("Starting variables");
	
	eepromRead(MeterCalibrationAdd, PulsesPerMeter);
	//eepromRead(LiterCalibrationAdd, LitersPerPulse);
	eepromRead(MachineWidthAdd, MachineWidth);
	eepromRead(LiterPerHaAdd, LiterPerHa);
	eepromRead(PulsesPerLiterAdd, PulsesPerLiter);
	LitersPerPulse = (1.00 / PulsesPerLiter) * pow(10, 6);

	TelemetrySerial.print("PPM: ");
	TelemetrySerial.println(PulsesPerMeter);
	TelemetrySerial.print("PPL: ");
	TelemetrySerial.println(PulsesPerLiter);
	TelemetrySerial.print("LPP: ");
	TelemetrySerial.println(LitersPerPulse);
	TelemetrySerial.print("MachineWidth: ");
	TelemetrySerial.println(MachineWidth);
	TelemetrySerial.print("LiterPerHa: ");
	TelemetrySerial.println(LiterPerHa);


	Speed.setup();
	Flux.setup();
	Controller.setup();

	TelemetrySerial.println("initialization done.");

}

void loop()
{

	//////////////////////////////////////////////////////////////////////////////////////////
	//Check for any user request /////////////////////////////////////////////////////////////
	/*
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

				PulsesPerMeter = Speed.calibrate();
				eepromWrite(MeterCalibrationAdd, PulsesPerMeter);

				TelemetrySerial.println(PulsesPerMeter);
				TelemetrySerial.println("SpeedCalibrate end");
				digitalWrite(SpeedLed, 0);
				MillisOld = millis();
				while(digitalRead(SpeedCalibrateButton));
			}
		}
	}
	*/
	//////////////////////////////////////////////////////////////////////////////////////////
	//Get Variables values and calculate all needed to run the PIDControl function////////////

	#ifdef SerialCom
		Scom.update();
		PulsesPerLiter = Scom.PulsesPerLiter();
		LitersPerPulse = (1.00 / PulsesPerLiter) * pow(10, 6);
	#endif

	float FluxLPM = Flux.update(LitersPerPulse);
	if(FluxLPM>30) FluxLPM=30;
	if(FluxLPM<1) FluxLPM=0;
	FluxLPM = Flux.average(FluxLPM, 5);


	float SpeedVal = 0;

	#ifdef SerialCom

		if(!Scom.acquisition())
		SpeedVal = Scom.Speed();
		
		Controller.proporcional(Scom.proporcional()/100);
		Controller.integrative(Scom.integrative()/100);
		Controller.derivative(Scom.derivative()/100);

	#else
		if(SpeedSimulationState) SpeedVal = SpeedSimulationVal;
		else{
			SpeedVal = Speed.update(PulsesPerMeter);
			SpeedVal = Speed.average(SpeedVal, 5);
			SpeedVal = constrain(SpeedVal, 0, MaxSpeedVal(LiterPerHa, MachineWidth));
			SpeedVal = KF_SpeedVal.updateEstimate(SpeedVal);
		}
	#endif

	float SetPointLPM = Calculate(LiterPerHa, SpeedVal, MachineWidth); 	//L/min, km/h, meters
	SetPointLPM = constrain(SetPointLPM, 0, 20);

	float setPoint = convert2SetPoint(SetPointLPM);              			//convert liter per min into setPoint, going from 0 to 1023(10bits)
	float FeedbackFlux = convert2SetPoint(FluxLPM);


	FeedbackFlux = KF_Flux.updateEstimate(FeedbackFlux);
	setPoint = KF_SetPoint.updateEstimate(setPoint);


	//########################################################################
	
	#ifdef SerialCom

			if(Scom.acquisition()){

				setPoint = loweracquisitionSetpoint;
				SpeedVal = MinSpeed+1;

				if(Controller.stability() && !acquisitionFirstStage){
					acquisitionFirstStage=true;
					acquisitionFirstRun=true;
					acquisitionStartTime=millis();
				}

				if(acquisitionFirstStage){
					
					setPoint=upperacquisitionSetpoint;

					if(!acquisitionFirstRun)
					if(Controller.stability()){
							Scom.stopacquisition();
							SpeedVal=0;
						}

					acquisitionFirstRun=false;	 
				}

			}
			else{

				acquisitionStartTime=millis();
				acquisitionFirstStage=false;
			}
	#endif
	//########################################################################3

	//////////////////////////////////////////////////////////////////////////////////////////////////
	//Apply Flux Control Routines/////////////////////////////////////////////////////////////////////

	bool minSpd = Speed.testMin(SpeedVal, MinSpeed);
	int dutyCycle = 0;

	if(minSpd  && !ValveState) //tests if the speed is higher then min speed and the valve is closed, then open it
	{

		TelemetrySerial.println("Opening Valve");
		Controller.openSession();
		ValveState = true; //indicates that the valve is now open

	}
	else if (!minSpd && ValveState) //tests if the speed is smaller then min speed and the valve is open, then close it
	{

		TelemetrySerial.println("Closing Valve");
		Controller.closeSession();
		Controller.releaseBridge();
		ValveState = false; //indicates that the valve is now closed
	}
	else if(minSpd && ValveState)
	{
		dutyCycle = Controller.controllerDuty();
		Controller.update(FeedbackFlux, setPoint);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////


	if(Scom.acquisition()){

	TelemetrySerial.print((millis()-acquisitionStartTime));
	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(setPoint);
	TelemetrySerial.print(F(", "));
	TelemetrySerial.print(FeedbackFlux);

	TelemetrySerial.println("");
	}
	else{
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
	TelemetrySerial.print(dutyCycle);

		#ifdef SerialCom
			TelemetrySerial.print(F(", "));
			TelemetrySerial.print(ValveState);

			TelemetrySerial.print(F(", "));
			TelemetrySerial.print(Controller.proporcional());
			TelemetrySerial.print(F(", "));
			TelemetrySerial.print(Controller.integrative());
			TelemetrySerial.print(F(", "));
			TelemetrySerial.print(Controller.derivative());

			TelemetrySerial.print(F(", "));
			TelemetrySerial.println(PulsesPerLiter);

		#else
			TelemetrySerial.print(F(", "));
			TelemetrySerial.println(ValveState);
		#endif

	#endif


	}
	

}
