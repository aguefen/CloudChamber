/*************************************************************
 * Cloud Chamber Controller
 * Using PWM control for the peltier modules
 * Using bit-banged simple on/off control for vapor resistors
 * Using Arduino Nano
 ************************************************************/

#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Peltier control pins must be PWM outputs
#define PIN_PELTIER_0 3
#define PIN_PELTIER_1 5
#define PIN_PELTIER_2 6
#define PIN_PELTIER_3 9

// Vapor resistor pins can be simple digtal outputs
#define PIN_VAPOR_0 0
#define PIN_VAPOR_1 1
#define PIN_VAPOR_2 2
#define PIN_VAPOR_3 4

//Temperature pin must be digital
#define PIN_ONEWIRE 20

//sensor number in onewire is determined by serial numbers of sensors
//easiest way to do this is touch sensor with hand and see which one gets warm, and route sensors according to that
int PELTIER_0_SENSOR=0, PELTIER_1_SENSOR=1, PELTIER_2_SENSOR=2, PELTIER_3_SENSOR=3;
int VAPOR_0_SENSOR=4,   VAPOR_1_SENSOR=5,   VAPOR_2_SENSOR=6,   VAPOR_3_SENSOR=7;

//onewire setup
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

//LED control digital in/out
#define PIN_LED_OUTPUT 7
#define PIN_LED_INPUT 8


//Define Variables we'll be connecting to
double Peltier_0_Setpoint, Peltier_0_Input, Peltier_0_Output;
double Peltier_1_Setpoint, Peltier_1_Input, Peltier_1_Output;
double Peltier_2_Setpoint, Peltier_2_Input, Peltier_2_Output;
double Peltier_3_Setpoint, Peltier_3_Input, Peltier_3_Output;

double Peltier_0_Kp=2,     Peltier_0_Ki=5,  Peltier_0_Kd=1;
double Peltier_1_Kp=2,     Peltier_1_Ki=5,  Peltier_1_Kd=1;
double Peltier_2_Kp=2,     Peltier_2_Ki=5,  Peltier_2_Kd=1;
double Peltier_3_Kp=2,     Peltier_3_Ki=5,  Peltier_3_Kd=1;

double Vapor_0_Setpoint,   Vapor_0_Input,  Vapor_0_Output;
double Vapor_1_Setpoint,   Vapor_1_Input,  Vapor_1_Output;
double Vapor_2_Setpoint,   Vapor_2_Input,  Vapor_2_Output;
double Vapor_3_Setpoint,   Vapor_3_Input,  Vapor_3_Output;

boolean LED_Input, LED_Output, LED_Input_History;

//setup PID instances
PID Peltier_0_PID(&Peltier_0_Input, &Peltier_0_Output, &Peltier_0_Setpoint, Peltier_0_Kp, Peltier_0_Ki, Peltier_0_Kd, DIRECT);
PID Peltier_1_PID(&Peltier_1_Input, &Peltier_1_Output, &Peltier_1_Setpoint, Peltier_1_Kp, Peltier_1_Ki, Peltier_1_Kd, DIRECT);
PID Peltier_2_PID(&Peltier_2_Input, &Peltier_2_Output, &Peltier_2_Setpoint, Peltier_2_Kp, Peltier_2_Ki, Peltier_2_Kd, DIRECT);
PID Peltier_3_PID(&Peltier_3_Input, &Peltier_3_Output, &Peltier_3_Setpoint, Peltier_3_Kp, Peltier_3_Ki, Peltier_3_Kd, DIRECT);



//Serial input variables
const byte Number_Chars = 32;
char Received_Chars[Number_Chars];

//variables to hold command data - commands come in the form fo X,Y,number
char Command_0[32] = {0};
char Command_1[32] = {0};
double Command_2 = 0.0;
boolean New_Data = false;

//Debug output variables
boolean Debug_Info = false;
int Debug_Rate = 360;
int Debug_Rate_Counter = 0;



void setup() {

  Serial.begin(9600);
  sensors.begin();

  Peltier_0_Setpoint = -35.00;
  Peltier_1_Setpoint = -35.00;
  Peltier_2_Setpoint = -35.00;
  Peltier_3_Setpoint = -35.00;

  Vapor_0_Setpoint = 20.00;
  Vapor_1_Setpoint = 20.00;
  Vapor_2_Setpoint = 20.00;
  Vapor_3_Setpoint = 20.00;

  LED_Input_History = 0;

  // turn PIDs on
  Peltier_0_PID.SetMode(AUTOMATIC);
  Peltier_1_PID.SetMode(AUTOMATIC);
  Peltier_2_PID.SetMode(AUTOMATIC);
  Peltier_3_PID.SetMode(AUTOMATIC);
  
  delay(1000);

}

void loop() {
  
  //only allow debug output every xth loop
  if (Debug_Rate>0){
    Debug_Rate_Counter++;
    if (Debug_Rate_Counter >= Debug_Rate){
      Debug_Info = true;
      Debug_Rate_Counter = 0;
    }
  }

  //Serial input
  RecvWithEndMarker();
  ParseSerialInput();

  LEDControl();

  //sensors.requestTemperatures(); //<-- currently broken until we actually have sensors.

  VaporController(VAPOR_0_SENSOR, PIN_VAPOR_0, Vapor_0_Setpoint, 0);
  VaporController(VAPOR_1_SENSOR, PIN_VAPOR_1, Vapor_1_Setpoint, 1);
  VaporController(VAPOR_2_SENSOR, PIN_VAPOR_2, Vapor_2_Setpoint, 2);
  VaporController(VAPOR_3_SENSOR, PIN_VAPOR_3, Vapor_3_Setpoint, 3);
  
  PeltierController();
  
  //tell how much memory we have left
  if (Debug_Info == true){
    Serial.println(String(availableMemory())+"B left in RAM\n");
  }

  Debug_Info = false;
  
}

// Read LED input switch and turn on/off LEDs
void LEDControl(){
  
  LED_Input = digitalRead(PIN_LED_INPUT);

  //Check if the input state changed
  if (LED_Input != LED_Input_History){
    
    if (LED_Input == 0){
      Serial.println("LEDs on->off\n");
      digitalWrite(PIN_LED_OUTPUT, LOW);
    }

    else{
      Serial.println("LEDs off->on\n");
      digitalWrite(PIN_LED_OUTPUT, HIGH);
    }

  }
  LED_Input_History = LED_Input;
}

//simple on/off control of vapor resistors
void VaporController(int Sensor_Number, int PIN_Output, int PIN_Setpoint, int PIN_Number){
  
  double Vapor_Temperature = MeasureTemperatureDS18B20(Sensor_Number);
  String Vapor_Response = "";
  
  if (Vapor_Temperature >= PIN_Setpoint){
    digitalWrite(PIN_Output, LOW);
    Vapor_Response += "V"+String(PIN_Number)+", PV = "+String(Vapor_Temperature)+"C, SV = "+String(PIN_Setpoint)+"C, Disabled";
  }

  else{
    digitalWrite(PIN_Output, HIGH);
    Vapor_Response += "V"+String(PIN_Number)+", PV = "+String(Vapor_Temperature)+"C, SV = "+String(PIN_Setpoint)+"C, Enabled";
  }

  if (Debug_Info == true && Vapor_Response != ""){
    Serial.println(Vapor_Response);
  }

}

//PWM control for Peltier modules
void PeltierController(){

  String Peltier_Response = "";

  //Measure actual temperatures
  Peltier_0_Input = MeasureTemperatureDS18B20(PELTIER_0_SENSOR);  
  Peltier_1_Input = MeasureTemperatureDS18B20(PELTIER_1_SENSOR);
  Peltier_2_Input = MeasureTemperatureDS18B20(PELTIER_2_SENSOR);
  Peltier_3_Input = MeasureTemperatureDS18B20(PELTIER_3_SENSOR);

  //Compute PWM Values
  Peltier_0_PID.Compute();
  Peltier_1_PID.Compute();
  Peltier_2_PID.Compute();
  Peltier_3_PID.Compute();

  //Write PWM Values, inverted (255-output) since the peltier modules cool (become more negative) with higher input
  //write this to a separate variable since when you call the output, it recalculates it for some reason
  double P_0_O = Peltier_0_Output;
  double P_1_O = Peltier_1_Output;
  double P_2_O = Peltier_2_Output;
  double P_3_O = Peltier_3_Output;

  //Write PWM values
  analogWrite(PIN_PELTIER_0, 255-P_0_O);
  analogWrite(PIN_PELTIER_1, 255-P_1_O);
  analogWrite(PIN_PELTIER_2, 255-P_2_O);
  analogWrite(PIN_PELTIER_3, 255-P_3_O);

  Peltier_Response +="P0, PV = "+String(Peltier_0_Input, 2)+"C, SV = "+String(Peltier_0_Setpoint, 2)+"C, PWM = "+String(255-P_0_O, 2)+" / "+String(100*(255-P_0_O)/255,2)+"%\n";
  Peltier_Response +="P1, PV = "+String(Peltier_1_Input, 2)+"C, SV = "+String(Peltier_1_Setpoint, 2)+"C, PWM = "+String(255-P_1_O, 2)+" / "+String(100*(255-P_1_O)/255,2)+"%\n";
  Peltier_Response +="P2, PV = "+String(Peltier_2_Input, 2)+"C, SV = "+String(Peltier_2_Setpoint, 2)+"C, PWM = "+String(255-P_2_O, 2)+" / "+String(100*(255-P_2_O)/255,2)+"%\n";
  Peltier_Response +="P3, PV = "+String(Peltier_3_Input, 2)+"C, SV = "+String(Peltier_3_Setpoint, 2)+"C, PWM = "+String(255-P_3_O, 2)+" / "+String(100*(255-P_3_O)/255,2)+"%\n";
 
  if (Debug_Info == true && Peltier_Response != ""){
    Serial.println(Peltier_Response);
  }
 
  }

//Measure Temperature from a DS18B20 temperature sensor
//https://cdn-shop.adafruit.com/datasheets/DS18B20.pdf
double MeasureTemperatureDS18B20(int Sensor_Number){
  //double Temp_Measurement = sensors.getTempCByIndex(Sensor_Number); //<- won't work until you have sensors
  //placeholder before having a real sensor
  double Temp_Measurement = random(-50, 50);
  return Temp_Measurement;
}


//Serial Input function
void RecvWithEndMarker() {
   
  static byte ndx = 0;
  char End_Marker = '\n';
  char rc;
    
  while (Serial.available() > 0 && New_Data == false) {
    rc = Serial.read();

    if (rc != End_Marker) {
      Received_Chars[ndx] = rc;
      ndx++;
      
      if (ndx >= Number_Chars) {
        ndx = Number_Chars - 1;
      }
        
    }

    else {
      Received_Chars[ndx] = '\0'; // terminate the string
      ndx = 0;
      New_Data = true;
    }

  }
}

void ParseSerialInput(){
  
  if (New_Data == true){
    Serial.println("Input: "+String(Received_Chars));
    New_Data = false;

    //count commas in input
    int count = 0;
    for (uint8_t i=0; i<strlen(Received_Chars); i++){
      if (Received_Chars[i] == ','){
         count++;
      }
    }

    if (count == 2){
      char * strtok_Index;
      char Command_2_Buffer[32] = {0};
      
      //Parse first section
      strtok_Index = strtok(Received_Chars,",");
      strcpy(Command_0, strtok_Index);

      //Parse second section
      strtok_Index = strtok(NULL,",");
      strcpy(Command_1,strtok_Index);

      //Parse third section
      strtok_Index = strtok(NULL,",");
      strcpy(Command_2_Buffer,strtok_Index);
      
      //Check to make sure third command is valid
      if (!isnan(atof(Command_2_Buffer))){
        
        //convert Command_2_Buffer to float, although this seems to not work since if Command_2_Buffer is just other characters, it returns 0 :(
        //not currently an issue
        Command_2 = atof(strtok_Index);
        String Response_String = "";

        //parse peltier / PID commands
        if(String(Command_0) == "P0"){
          Response_String+="Peltier 0 ";
          
          if(String(Command_1) == "Kp"){
            Response_String+="Kp set to "+String(Command_2);
            Peltier_0_Kp = Command_2;
          }
          
          if(String(Command_1) == "Ki"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_0_Ki = Command_2;
          }
          
          if(String(Command_1) == "Kd"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_0_Kd = Command_2;
          }
          
          if(String(Command_1) == "T"){
            Response_String+="SV set to "+String(Command_2)+"C";
            Peltier_0_Setpoint = Command_2;
          }
        }

        if(String(Command_0) == "P1"){
          Response_String+="Peltier 1 ";
          
          if(String(Command_1) == "Kp"){
            Response_String+="Kp set to "+String(Command_2);
            Peltier_1_Kp = Command_2;
          }
          
          if(String(Command_1) == "Ki"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_1_Ki = Command_2;
          }
          
          if(String(Command_1) == "Kd"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_1_Kd = Command_2;
          }
          
          if(String(Command_1) == "T"){
            Response_String+="SV set to "+String(Command_2)+"C";
            Peltier_1_Setpoint = Command_2;
          }
        }

        if(String(Command_0) == "P2"){
          Response_String+="Peltier 2 ";
          
          if(String(Command_1) == "Kp"){
            Response_String+="Kp set to "+String(Command_2);
            Peltier_2_Kp = Command_2;
          }
          
          if(String(Command_1) == "Ki"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_2_Ki = Command_2;
          }
          
          if(String(Command_1) == "Kd"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_2_Kd = Command_2;
          }
          
          if(String(Command_1) == "T"){
            Response_String+="SV set to "+String(Command_2)+"C";
            Peltier_2_Setpoint = Command_2;
          }
        }

        if(String(Command_0) == "P3"){
          Response_String+="Peltier 3 ";
          
          if(String(Command_1) == "Kp"){
            Response_String+="Kp set to "+String(Command_2);
            Peltier_3_Kp = Command_2;
          }
          
          if(String(Command_1) == "Ki"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_3_Ki = Command_2;
          }
          
          if(String(Command_1) == "Kd"){
            Response_String+="Ki set to "+String(Command_2);
            Peltier_3_Kd = Command_2;
          }
          
          if(String(Command_1) == "T"){
            Response_String+="SV set to "+String(Command_2)+"C";
            Peltier_3_Setpoint = Command_2;
          }
        }

        //parse vapor controller commands
        if(String(Command_0) == "V0"){
          Vapor_0_Setpoint = Command_2;
          Response_String+="Vapor 0 SV set to "+String(Command_2)+"C";
        }
        
        if(String(Command_0) == "V1"){
          Vapor_1_Setpoint = Command_2;
          Response_String+="Vapor 1 SV set to "+String(Command_2)+"C";
        }
        
        if(String(Command_0) == "V2"){
          Vapor_2_Setpoint = Command_2;
          Response_String+="Vapor 2 SV set to "+String(Command_2)+"C";
        }
        
        if(String(Command_0) == "V3"){
          Vapor_3_Setpoint = Command_2;
          Response_String+="Vapor 3 SV set to "+String(Command_2)+"C";
        }


        //parse debug rate commands
        if(String(Command_0) == "D"){
          Debug_Rate = Command_2;
          Response_String+="Debug Rate set to "+String(Command_2);
        }

        Serial.println(Response_String+"\n");
        Serial.println("Command_0: "+String(Command_0));
        Serial.println("Command_1: "+String(Command_1));
        Serial.println("Command_2: "+String(Command_2)+"\n");
        
      }
      else{
        Serial.println("Command_2: '"+String(Command_2_Buffer)+"' is not a float");
      }
      


    }
  
    else if (count == 0 && String(Received_Chars) == "?"){
      Serial.println("Command structure:");
      Serial.println("Example - Setting Peltier 0 Ki value to 3:");
      Serial.println("P0,Ki,3");
      Serial.println("Example - Setting Vapor 3 setpoint value to 32.50C:");
      Serial.println("V3,T,32.5");
      Serial.println("Example - Setting debug rate value to 600:");
      Serial.println("D,D,600");
      Serial.println("Example - Disable debug output:");
      Serial.println("D,D,-1\n");
      
    }
    else{
      Serial.println("Command: '"+String(Received_Chars)+"' is Invalid");
    }
    
  }
}


// free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
int availableMemory() {
    // Use 1024 with ATmega168
    int size = 2048;
    byte *buf;
    while ((buf = (byte *) malloc(--size)) == NULL);
        free(buf);
    return size;
}