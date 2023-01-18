#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x60)

#define OR_MOSFET 11U
#define OR_GLED 6U
#define OR_RLED 12U
#define OR_SW 4U

#define PID_Kp 75
#define PID_Ki 5
#define PID_Kd 30

uint16_t OR_RTempArray[271U] = {20U,  20U,  21U,  22U,  23U,  23U,  24U,  25U,  26U,  27U,  27U,  28U,  29U,  30U,  30U,  31U,  32U,  33U,  34U,  34U,  35U,  36U,  37U,  37U,  38U,  39U,  40U,  41U,  41U,  42U,  43U,  44U,  44U,  45U,  46U,  47U,  48U,  48U,  49U,  50U,  51U,  51U,  52U,  53U,  54U,  55U,  55U,  56U,  57U,  58U,  58U,  59U,  60U,  61U,  62U,  62U,  63U,  64U,  65U,  65U,  66U,  67U,  68U,  69U,  69U,  70U,  71U,  72U,  72U,  73U,  74U,  75U,  76U,  76U,  77U,  78U,  79U,  79U,  80U,  81U,  82U,  83U,  83U,  84U,  85U,  86U,  86U,  87U,  88U,  89U,  90U,  90U,  90U,  91U,  91U,  92U,  92U,  93U,  93U,  93U,  94U,  94U,  95U,  95U,  96U,  96U,  97U,  97U,  97U,  98U,  98U,  99U,  99U,  100U, 100U, 101U, 101U, 101U, 102U, 102U, 103U, 103U, 104U, 104U, 105U, 105U, 105U, 106U, 106U, 107U, 107U, 108U, 108U, 109U, 109U, 109U, 110U, 110U, 111U, 111U, 112U, 112U, 113U, 113U, 113U, 114U, 114U, 115U, 115U, 116U, 116U, 117U, 117U, 117U, 118U, 118U, 119U, 119U, 120U, 120U, 121U, 121U, 121U, 122U, 122U, 123U, 123U, 124U, 124U, 125U, 125U, 125U, 126U, 126U, 127U, 127U, 128U, 128U, 129U, 129U, 130U, 130U, 130U, 130U, 131U, 131U, 131U, 131U, 132U, 132U, 132U, 132U, 133U, 133U, 133U, 134U, 134U, 134U, 134U, 135U, 135U, 135U, 135U, 136U, 136U, 136U, 136U, 137U, 137U, 137U, 138U, 138U, 139U, 140U, 141U, 142U, 143U, 144U, 145U, 146U, 147U, 147U, 148U, 149U, 150U, 151U, 152U, 153U, 154U, 155U, 156U, 156U, 157U, 158U, 159U, 160U, 161U, 162U, 163U, 164U, 165U, 164U, 163U, 162U, 161U, 160U, 159U, 158U, 157U, 156U, 156U, 155U, 154U, 153U, 152U, 151U, 150U, 149U, 148U, 147U, 147U, 146U, 145U, 144U, 143U, 142U, 141U, 140U, 139U, 138U, 138U};

Adafruit_MCP9600 mcp;
bool OR_Flag = false;
bool LED_flag = false;
double OR_Time0 = 0.0, OR_Time = 0.0, PID_TimeChange = 0.0, PID_TimeLast  = 0.0, PID_Error = 0.0, PID_Intg = 0.0, PID_Derv = 0.0, PID_ErrorLast = 0.0;
uint16_t OR_TimeULast = 0U;

void setup() {
  pinMode(OR_MOSFET, OUTPUT);
  pinMode(OR_GLED, OUTPUT);
  pinMode(OR_RLED, OUTPUT);
  pinMode(OR_SW, INPUT);
  
  analogWrite(OR_RLED, 0); /*Disable PWM*/
  analogWrite(OR_MOSFET, 0); /*Disable PWM*/
    
  Serial.begin(19200);
//  while (!Serial){
//     delay(10);
//  }
  /*while the serial stream is not open, do nothing*/
  Serial.println("OPEN REFLOW");
  Serial.println("Starting Up the Board");
  
  analogWrite(OR_RLED, 200); /*Disable PWM*/
  delay(500);
  if(! mcp.begin(I2C_ADDRESS)){
    Serial.println("Sensor not found. Check wiring!");
    while(1){}
  }
  else{
    Serial.println("Sensor Found!");
  }

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp.setThermocoupleType(MCP9600_TYPE_K);
  mcp.setFilterCoefficient(3);
  mcp.enable(true);

  Serial.print("Ambient Temperature(°C):"); Serial.println(mcp.readAmbient());

  analogWrite(OR_RLED, 0); /*Disable PWM*/      
  digitalWrite(OR_GLED, HIGH); 
}

void loop() {
  double OR_Temp = mcp.readThermocouple();
  double OR_Ambient = mcp.readAmbient();
  double OR_RTemp = 0.0;
  double OR_ROut = 0U;
         
  if((!digitalRead(OR_SW))||(OR_Flag == true)){    
      digitalWrite(OR_GLED, LOW);   
      delay(1);
      if(OR_Flag == false){
        OR_Time0 = ((double)millis())/1000;/*Read intial time*/
        OR_Flag = 1;
      }
      else{
        /*SPID_Kip*/
      }        

      OR_Time = (((double)millis())/1000)-OR_Time0;/*Read current time*/       
      uint16_t OR_TimeU = (uint16_t)OR_Time;/*convert to integer*/

      if (OR_TimeU >= 270U){
        OR_Flag = 0; /*Cool off*/           
      }      
      else{
        OR_RTemp = OR_RTempArray[OR_TimeU];/*Get desired Temperature*/
      }
      if(OR_TimeU != OR_TimeULast){
        OR_TimeULast = OR_TimeU;
        Serial.print("Board Temperature:"); Serial.println(OR_Temp);/*Print current temperature of the board*/
        Serial.print("Desired Temperature:"); Serial.println(OR_RTemp); /*Print desired temperature of the board*/
        //Serial.print("Output:"); Serial.println((int)OR_ROut);
        //Serial.print("Delta:"); Serial.println(OR_RTemp-OR_Temp);
      }
      else{
        /*Skip*/
      }
      
      /*Compute PID on sensed temperature*/
      PID_TimeChange = OR_Time - PID_TimeLast;    
      PID_Error = OR_RTemp - OR_Temp;
      PID_Intg = PID_Intg + (PID_Error * PID_TimeChange);
      PID_Derv = (PID_Error - PID_ErrorLast)/PID_TimeChange;
      OR_ROut = (PID_Kp*PID_Error) + (PID_Ki*PID_Intg) + (PID_Kd*PID_Derv);              
      if(OR_ROut > 255.0){
        OR_ROut = 255.0;
      }
      if(OR_ROut < 0.0){
        OR_ROut = 0.0;
      }  
      PID_ErrorLast = PID_Error;
      PID_TimeLast = OR_Time; 
       
      analogWrite(OR_MOSFET, (int)OR_ROut);
      analogWrite(OR_RLED, (int)OR_ROut); 
  }
  else{
    /*wait*/
      analogWrite(OR_RLED, 0); /*Disable PWM*/
    analogWrite(OR_MOSFET, 0); /*Disable PWM*/
    delay(500);
    if(OR_Temp >= (OR_Ambient + 2.0)){  
      Serial.print("Board cooling off. Temperature at "); Serial.print(OR_Temp); Serial.print("°C Waiting to reach "); Serial.print(OR_Ambient); Serial.println("°C"); 
      delay(500);
      digitalWrite(OR_GLED, HIGH); 
      delay(500);
      digitalWrite(OR_GLED, LOW); 
    }
    else{
      delay(1000);
      if(!digitalRead(OR_GLED)){
        digitalWrite(OR_GLED, HIGH); 
        Serial.println("Press START to Reflow");
        PID_Intg = 0;
        PID_Error = 0;
        PID_Derv = 0;
        PID_ErrorLast = 0;
        PID_TimeLast = 0;
      }
      else{
        /*Wait*/
      }
    }    
  }
}
