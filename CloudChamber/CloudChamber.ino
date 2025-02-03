/*************************************************************
 * Cloud Chamber Controller
 * Using PWM control for the peltier modules
 * Using bit-banged simple on/off control for vapor resistors
 *Using Arduino Nano
 ************************************************************/

#include <PID_v1.h>

// Peltier control pins must be PWM outputs
#define PIN_PELTIER_1 3
#define PIN_PELTIER_2 5
#define PIN_PELTIER_3 6
#define PIN_PELTIER_4 9

// Vapor resistor pins can be simple digtal outputs
#define PIN_VAPOR_1 0
#define PIN_VAPOR_2 1
#define PIN_VAPOR_3 2
#define PIN_VAPOR_4 4

// Temperature pins must be analog inputs
#define PIN_PELTIER_1_TEMP A0
#define PIN_PELTIER_2_TEMP A1
#define PIN_PELTIER_3_TEMP A2
#define PIN_PELTIER_4_TEMP A3

#define PIN_VAPOR_1_TEMP A4
#define PIN_VAPOR_2_TEMP A5
#define PIN_VAPOR_3_TEMP A6
#define PIN_VAPOR_4_TEMP A7


#define PIN_LED_OUTPUT 7
#define PIN_LED_INPUT 8


//Define Variables we'll be connecting to
double Peltier_1_Setpoint, Peltier_1_Input, Peltier_1_Output;
double Peltier_2_Setpoint, Peltier_2_Input, Peltier_2_Output;
double Peltier_3_Setpoint, Peltier_3_Input, Peltier_3_Output;
double Peltier_4_Setpoint, Peltier_4_Input, Peltier_4_Output;

double Peltier_1_Kp=2, Peltier_1_Ki=5, Peltier_1_Kd=1;
double Peltier_2_Kp=2, Peltier_2_Ki=5, Peltier_2_Kd=1;
double Peltier_3_Kp=2, Peltier_3_Ki=5, Peltier_3_Kd=1;
double Peltier_4_Kp=2, Peltier_4_Ki=5, Peltier_4_Kd=1;

double Vapor_1_Setpoint, Vapor_1_Input, Vapor_1_Output;
double Vapor_2_Setpoint, Vapor_2_Input, Vapor_2_Output;
double Vapor_3_Setpoint, Vapor_3_Input, Vapor_3_Output;
double Vapor_4_Setpoint, Vapor_4_Input, Vapor_4_Output;

int LED_Input, LED_Output, LED_Input_History;

PID Peltier_1_PID(&Peltier_1_Input, &Peltier_1_Output, &Peltier_1_Setpoint, Peltier_1_Kp, Peltier_1_Ki, Peltier_1_Kd, DIRECT);
PID Peltier_2_PID(&Peltier_2_Input, &Peltier_2_Output, &Peltier_2_Setpoint, Peltier_2_Kp, Peltier_2_Ki, Peltier_2_Kd, DIRECT);
PID Peltier_3_PID(&Peltier_3_Input, &Peltier_3_Output, &Peltier_3_Setpoint, Peltier_3_Kp, Peltier_3_Ki, Peltier_3_Kd, DIRECT);
PID Peltier_4_PID(&Peltier_4_Input, &Peltier_4_Output, &Peltier_4_Setpoint, Peltier_4_Kp, Peltier_4_Ki, Peltier_4_Kd, DIRECT);


void setup() {

  Serial.begin(9600);

  Peltier_1_Input = MeasureTemperatureDS18B20(PIN_PELTIER_1_TEMP);
  Peltier_2_Input = MeasureTemperatureDS18B20(PIN_PELTIER_2_TEMP);
  Peltier_3_Input = MeasureTemperatureDS18B20(PIN_PELTIER_3_TEMP);
  Peltier_4_Input = MeasureTemperatureDS18B20(PIN_PELTIER_4_TEMP);

  Peltier_1_Setpoint = -35.00;
  Peltier_2_Setpoint = -35.00;
  Peltier_3_Setpoint = -35.00;
  Peltier_4_Setpoint = -35.00;

  Vapor_1_Setpoint = 20.00;
  Vapor_2_Setpoint = 20.00;
  Vapor_3_Setpoint = 20.00;
  Vapor_4_Setpoint = 20.00;

  LED_Input_History = 0;

  // turn PIDs on
  Peltier_1_PID.SetMode(AUTOMATIC);
  Peltier_2_PID.SetMode(AUTOMATIC);
  Peltier_3_PID.SetMode(AUTOMATIC);
  Peltier_4_PID.SetMode(AUTOMATIC);
  
  delay(2000);

}

void loop() {
 
  LEDControl();

  VaporController(PIN_VAPOR_1_TEMP, PIN_VAPOR_1, Vapor_1_Setpoint, 1);
  VaporController(PIN_VAPOR_2_TEMP, PIN_VAPOR_2, Vapor_1_Setpoint, 2);
  VaporController(PIN_VAPOR_3_TEMP, PIN_VAPOR_3, Vapor_1_Setpoint, 3);
  VaporController(PIN_VAPOR_4_TEMP, PIN_VAPOR_4, Vapor_1_Setpoint, 4);
  Serial.println();

  PeltierController();
  Serial.println();
  
}

// Read LED input switch and turn on/off LEDs
void LEDControl(){
  LED_Input = digitalRead(PIN_LED_INPUT);

  //Check if the input state changed
  if (LED_Input != LED_Input_History){
    if (LED_Input == 0){
      Serial.println("LEDs were on, turning them off");
      digitalWrite(PIN_LED_OUTPUT, LOW);
    }
    else{
      Serial.println("LEDs were off, turning them on");
      digitalWrite(PIN_LED_OUTPUT, HIGH);
    }
    Serial.println();
  }
  LED_Input_History = LED_Input;
}

//simple on/off control of vapor resistors
void VaporController(int PIN_Input, int PIN_Output, int PIN_Setpoint, int PIN_Number){
  double Vapor_Temperature = MeasureTemperatureDS18B20(PIN_Input);

  if (Vapor_Temperature >= PIN_Setpoint){
    digitalWrite(PIN_Output, LOW);
    Serial.println("Vapor "+String(PIN_Number)+",   PV = "+String(Vapor_Temperature)+"C, SV = "+String(PIN_Setpoint)+"C. Output Disabled");
  }
  else{
    digitalWrite(PIN_Output, HIGH);
    Serial.println("Vapor "+String(PIN_Number)+",   PV = "+String(Vapor_Temperature)+"C, SV = "+String(PIN_Setpoint)+"C. Output Enabled");
  }


}

//PWM control for Peltier modules
void PeltierController(){

  //Measure actual temperatures
  Peltier_1_Input = MeasureTemperatureDS18B20(PIN_PELTIER_1_TEMP);
  Peltier_2_Input = MeasureTemperatureDS18B20(PIN_PELTIER_2_TEMP);
  Peltier_3_Input = MeasureTemperatureDS18B20(PIN_PELTIER_3_TEMP);
  Peltier_4_Input = MeasureTemperatureDS18B20(PIN_PELTIER_4_TEMP);
  
  //Compute PWM Values
  Peltier_1_PID.Compute();
  Peltier_2_PID.Compute();
  Peltier_3_PID.Compute();
  Peltier_4_PID.Compute();

  //Write PWM Values, inverted (255-output) since the peltier modules cool (become more negative) with higher input
  analogWrite(PIN_PELTIER_1, 255-Peltier_1_Output);
  analogWrite(PIN_PELTIER_2, 255-Peltier_2_Output);
  analogWrite(PIN_PELTIER_3, 255-Peltier_3_Output);
  analogWrite(PIN_PELTIER_4, 255-Peltier_4_Output);

  Serial.println("Peltier 1, PV = "+String(Peltier_1_Input, 2)+"C, SV = "+String(Peltier_1_Setpoint, 2)+"C, PWM Value = "+String(255-Peltier_1_Output, 2));
  Serial.println("Peltier 2, PV = "+String(Peltier_2_Input, 2)+"C, SV = "+String(Peltier_2_Setpoint, 2)+"C, PWM Value = "+String(255-Peltier_2_Output, 2));
  Serial.println("Peltier 3, PV = "+String(Peltier_3_Input, 2)+"C, SV = "+String(Peltier_3_Setpoint, 2)+"C, PWM Value = "+String(255-Peltier_3_Output, 2));
  Serial.println("Peltier 4, PV = "+String(Peltier_4_Input, 2)+"C, SV = "+String(Peltier_4_Setpoint, 2)+"C, PWM Value = "+String(255-Peltier_4_Output, 2));

  }

//Measure Temperature from a DS18B20 temperature sensor
double MeasureTemperatureDS18B20(int PIN_Input){
  double Temp_Measurement = analogRead(PIN_Input);
  
  //placeholder before having a real sensor
  Temp_Measurement = random(-50, 50);
  return Temp_Measurement;
}
