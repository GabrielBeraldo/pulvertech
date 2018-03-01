//Wiper motor software v 1.3:
//Servo sensor to Analog 0
//Setpoint potentiometer to Analog 1
//H bridge direction pin to Digital 2
//H bridge PWM signal to Digital 3
//
//For the first test run, set MAX_DUTYCYCLE to 75 in order to lower the maximum torque in case something goes wrong.
//
//#include <avr/io.h>
//#include <util/delay.h>


int ADC_SetPoint = 0;
int ADC_SetPointOld = 0;
int ADC_ServoPoti = 0;
int ADC_ServoPotiOld = 0;
int dutyCycle = 0; // 10 - 255
int ADCdiff = 0;
int timeDiff = 0;

//Change values below to adapt your motor
//Set MAX_DUTYCYCLE to 75 for the first test run!

#define P_FRACTION 0.3         //0.0 - 10.0 (0.3)
#define I_FRACTION 0.3         //0.0 - 10.0 (0.3)
#define D_FRACTION 2,9         //0.0 - 10.0 (4.0)

#define V_WINDOW 25            //10 - 1000 (25)
#define MIN_DUTYCYCLE 25       //0 - 255 (25)
#define MAX_DUTYCYCLE 255      //0 - 255 (255)
#define SOFT_START 0.3         //0.00 - 1.00 (0.30) 1.00 = OFF
#define EMERGENCY_SHUTDOWN 4   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE 10 //Prevent H bridge from shoot through whenever the direction pin is changed

#define SERVO_DIRECTION_PIN 2  //Direction pin servo motor
#define SERVO_PWM_PIN 3        //PWM pin servo motor
#define SERVO_SENSOR A0        //Analog input servo sensor
#define POTI A1                //Analog input potentiometer



void setup(){
  pinMode(SERVO_DIRECTION_PIN, OUTPUT);     
  pinMode(SERVO_PWM_PIN, OUTPUT);     

  pinMode(SERVO_SENSOR, INPUT);     
  pinMode(POTI, INPUT);     

  Serial.begin(9600);  
  
} 

 
void loop(){ 
  ADC_ServoPoti = analogRead(0);     // reads the servo sensor (between 0 and 1023) 
  ADC_SetPoint = analogRead(1);             
    
  ADCdiff = ADC_SetPoint - ADC_ServoPoti;
  
  //PID implenetatiom
  dutyCycle = abs(ADCdiff) * P_FRACTION;
  dutyCycle += timeDiff * I_FRACTION;
  dutyCycle += abs(ADC_SetPointOld - ADC_SetPoint) * D_FRACTION;
  //end of calculus of error on PID
  
  
 
  if(SOFT_START * timeDiff < 1){
    dutyCycle = dutyCycle * (SOFT_START * timeDiff);
  }
  
  timeDiff++;
  
  if(dutyCycle < MIN_DUTYCYCLE && dutyCycle > 0){
    dutyCycle = MIN_DUTYCYCLE;
  }
  
  if(dutyCycle > MAX_DUTYCYCLE){
    dutyCycle = MAX_DUTYCYCLE;
  }
  
  if(dutyCycle < 0){
    dutyCycle = 0;
  }
  

    if(abs(ADCdiff) < V_WINDOW){
      dutyCycle = 0;
      timeDiff = 0;
    }

    if(abs(ADC_ServoPotiOld - ADC_ServoPoti) < EMERGENCY_SHUTDOWN && dutyCycle == MAX_DUTYCYCLE && timeDiff > 50){
      analogWrite(SERVO_PWM_PIN, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE);
      digitalWrite(SERVO_DIRECTION_PIN, 0);
      delay(1000);
      timeDiff = 0;
    }
    else{  
      if(ADCdiff > 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
      if(ADCdiff < 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN, 1);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
    }
  
  

  ADC_SetPointOld = ADC_SetPoint;
  ADC_ServoPotiOld = ADC_ServoPoti;

 
Serial.print(ADC_ServoPoti);
Serial.print(" ");
Serial.println(ADC_SetPoint);

 // Serial.println(analysis);

  delay(15);                             // waits for the servo to get there 

}



