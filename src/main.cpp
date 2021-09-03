#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#define BLINK_RATE 500L      //Change onboard led blink rate (Must have L)
#define Debug Serial.println //quick statement debug

#define Blue 19 //RGB Lights for Temperature set vs Actual
#define Green 21
#define Red 22

//Motor Defines
#define stepPin 26
#define stepsPerRevolution 200

// PWM Pin
#define PWM_Pin 27

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Arduino.h>
#include "Ticker.h"
#include "max6675.h"
#include <analogWrite.h>
#include <AccelStepper.h>

//pre-declared functions (needed for Tickers)
void HotEndControl();

//ACCEL LIBRARY
AccelStepper stepper(AccelStepper::FULL2WIRE, 15, stepPin);

//Ticker Timers for independent non-blocking
Ticker Timer1(HotEndControl, 250);

//Blynk Timers - Form seperate timers to the Ticker library
BlynkTimer Pulse1, Pulse2, Pulse3;

//Independent Variables ---------------------------------------------------------------------
//Boolean
bool state = HIGH;  //binary 1 or 0x01
bool Heatpin = LOW; //Heating Mode On / Off
bool RST = 0;
bool BJT_STATE = 1;

//Integers
uint16_t MotorSpeed = 0;
int8_t BLINK_CONDITION = 1;
int16_t TEMP = 0;         //int 16 bits
int16_t PWM = 0;

//floats
float T = 0;
float Set_Temperature = 0;

//Constant ints - will not change; save space
const int thermoDO = 13;
const int thermoCS = 12;
const int thermoCLK = 14;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "_Jo-e2fTo4BuQpLF22Ildt6HIpmblvbo";

//Temperature sensor object configuration
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//Blynk Device Interface Instances
BLYNK_WRITE(V0)
{
  MotorSpeed = param.asInt();
}

BLYNK_WRITE(V1)
{
  Set_Temperature = param.asFloat();
}

BLYNK_WRITE(V2)
{
  Heatpin = param.asInt();
}

BLYNK_WRITE(V10)
{
  RST = param.asInt();

  if (RST == 1)
    BLINK_CONDITION = 0;
  delay(2000);
  ESP.restart();
}

void RGB_FOR_HEATER(int8_t CONDITION)
{
  switch (CONDITION)
  {
  case 1: // Lower
    digitalWrite(Blue, HIGH);
    digitalWrite(Green, LOW);
    digitalWrite(Red, LOW);
    break;

  case 2: //Good Range
    digitalWrite(Blue, LOW);
    digitalWrite(Green, HIGH);
    digitalWrite(Red, LOW);
    break;

  case 3: //Higher
    digitalWrite(Blue, LOW);
    digitalWrite(Green, LOW);
    digitalWrite(Red, HIGH);
    break;

  case 4: // Heater Off
    digitalWrite(Blue, LOW);
    digitalWrite(Green, LOW);
    digitalWrite(Red, LOW);
    break;

  default: //default
    digitalWrite(Blue, HIGH);
    digitalWrite(Green, HIGH);
    digitalWrite(Red, LOW);
    break;
  }
}

void LightToggle()
{
  Debug("Motor Speed is : ");
  Debug(MotorSpeed);
  switch (BLINK_CONDITION)
  {
  case 0:
    Pulse1.setInterval(BLINK_RATE / 3, LightToggle);
    digitalWrite(2, state);
    state = !state;

  default:
    digitalWrite(2, state);
    state = !state;
    break;
  }
}

void PWM_FUNCTION(int BJT_STATE)
{
  switch (BJT_STATE)
  {
  case 0:
    Debug("PWM Set to: ");
    Debug(PWM);

    Debug(0);
    break;

  case 1:

    analogWrite(PWM_Pin, 1023, 1023);
    Debug("PWM Set to: ");
    Debug(64);
    break;

  default:
    Debug("PWM Set to: ");
    Debug(1023);
    analogWrite(PWM_Pin, 0, 1023);
    break;
  }
}

void Temperature()
{
  T = thermocouple.readCelsius();

  Serial.print("C = ");
  Serial.println(thermocouple.readCelsius());
  Serial.print("F = ");
  Serial.println(thermocouple.readFahrenheit());
  Blynk.virtualWrite(V3, thermocouple.readCelsius());
  Blynk.virtualWrite(V4, thermocouple.readFahrenheit());
}

void HotEndControl()
{
  Serial.print("Hot End Function: ");
  Serial.println(Heatpin);

  if (Heatpin == HIGH)
  {
    if (T < (Set_Temperature + 2))
    {
      RGB_FOR_HEATER(1);
      Debug("Set_Temperature is: ");
      Debug(Set_Temperature);
      Debug("T is: ");
      Debug(T);
      Debug("Lower Than Set--Raise!");
      PWM_FUNCTION(0);
      Debug("MOSFET FULLY ON! \n BJT OFF");
    }

    if (T > (Set_Temperature - 2))
    {
      RGB_FOR_HEATER(3);
      Debug("Set_Temperature is: ");
      Debug(Set_Temperature);
      Debug("T is: ");
      Debug(T);
      Debug("Higher Than Set--Reduce!");
      PWM_FUNCTION(1);
      Debug("MOSFET FULLY OFF! \n BJT ON");
    }

    if (T > (Set_Temperature - 2) && T < (Set_Temperature + 2))
    {
      RGB_FOR_HEATER(2);
      Debug("Set_Temperature is: ");
      Debug(Set_Temperature);
      Debug("T is: ");
      Debug(T);
      Debug("Good Range!");
      PWM_FUNCTION(-1);
      Debug("MOSFET IDLE OFF! \n BJT IDLE");
    }
  }

  else
  {
    RGB_FOR_HEATER(0);
    Debug("Heater Off: Exit Sucessful");
  }
}

void Motor()
{
  stepper.setSpeed(MotorSpeed);
  stepper.runSpeed();
}

void setup()
{

  pinMode(2, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(Blue, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(Green, OUTPUT);

  analogWriteResolution(PWM_Pin, 12);
  analogWriteFrequency(PWM_Pin, 1);

  Timer1.start();

  // Debug console
  Serial.begin(115200); //default

  Pulse1.setInterval(BLINK_RATE, LightToggle);
  Pulse2.setInterval(500L, Temperature);
  Pulse3.setInterval(2L, Motor);

  //Pulse3.setInterval(250L, PWM_FUNCTION);

  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("Blynk");

  Blynk.begin(auth);

  stepper.setMaxSpeed(500);

  delay(3000);
}

void loop()
{
  Timer1.update(); //Light
  Pulse1.run();    //Blinker
  Pulse2.run();    //Temperature
  Pulse3.run(); //Motor
  Blynk.run();
}
