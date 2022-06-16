#include <Arduino.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include "ACPID.h"

#ifndef LOGGING
  #define LOGGING 0
  // 0 NONE
  // 1 FATAL
  // 2 ERROR
  // 3 WARNING
  // 4 INFO
  // 5 DEBUG
  // 6 TRACE
#endif

#ifndef TEST
  #define TEST false
#endif


//************************ add ArduinoLog and remove PID library


//pin declarations
#define Hall_Sensor_Pin A0
const uint8_t zero_cross_pin = 3;    //zero cross pin for hardware interrupt
const uint8_t SPI_clock = 52;        //for thermocouples
const uint8_t SPI_MISO = 50;
const uint8_t SPI_thermoA = 10;      
const uint8_t SPI_thermoB = 11;
const uint8_t SPI_thermoC = 12;
// PULSE PIN 22, 23, 24


//constants
const int pulse_delay_max = 16600;
bool zero_cross = false;
const int Delay_readtemp = 500;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
bool read_loop = false;


//classes init
//set_temp, kP, kI, kD, reversed direction
ACPID heaterA(90, 2, 2, 2, true);    
ACPID heaterB(90, 2, 2, 2, true);           
ACPID heaterC(90, 2, 2, 2, true);         

MAX6675 thermoA(SPI_clock, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_clock, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_clock, SPI_thermoC, SPI_MISO);

LiquidCrystal_I2C lcd(0x27,20,4);


//function declarations
void reset_timer();
void heater_loop();

//*************** add void PID_compute_routine();


void setup() {
  #if LOGGING > 0
  Serial.begin(9600);   //for testing
  #endif
  
  
  cli(); //stops interrupts

  //sets PA0 to PA2 (pin 22-24) as output
  DDRA |= B00000111;

  //sets timer 4
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= B00000010;  //set prescaler to 8
  TIMSK4 |= B00001110;  //enable compare match 4A, 4B, 4C for heaters

  //sets timer 5
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= B00000010;  //set prescaler to 8
  TIMSK5 |= B00000010;  //enable compare match 5A for timer reset 

  //firing delays rising edge
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  //firing delays falling edge
  OCR5A = pulse_delay_max;

  sei();  //continue interrupts
  pinMode(zero_cross_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_cross_pin), reset_timer, RISING);

  //lcd initialization
  lcd.init();
  lcd.backlight();
  
  //PID settings
  heaterA.Range(600,16660);
  heaterB.Range(600,16660);
  heaterC.Range(600,16660);

  // hall sensor
  pinMode(Hall_Sensor_Pin,INPUT);
}

void reset_timer(){
  PORTA &= !B00000111; 
  zero_cross = true;
  TCNT4 = 0;
  TCNT5 = 0;
}

void heater_loop(){
  previousMillis += Delay_readtemp;

  noInterrupts();
  heaterA.Input = thermoA.readCelsius();    //store temp reading to ACPID.Input
  heaterB.Input = thermoB.readCelsius();
  heaterC.Input = thermoC.readCelsius();
  interrupts();

  heaterA.Compute(Delay_readtemp);
  heaterB.Compute(Delay_readtemp);
  heaterC.Compute(Delay_readtemp);

  OCR4A = heaterA.Pulse_Delay;
  OCR4B = heaterB.Pulse_Delay;
  OCR4C = heaterC.Pulse_Delay;

  #if LOGGING >= 5 //DEBUG 5
    // Serial.println(OCR4A);
    // Serial.println(OCR4B);
    Serial.println(OCR4C);
  #endif

  
  lcd.setCursor(0,0);
  lcd.print("Set: ");
  lcd.setCursor(10,0);
  lcd.print("Real: ");

  lcd.setCursor(0,1);
  lcd.print(heaterA.Setpoint);
  lcd.setCursor(10,1);
  lcd.print(heaterA.Input);

  lcd.setCursor(0,2);
  lcd.print(heaterB.Setpoint);
  lcd.setCursor(10,2);
  lcd.print(heaterB.Input);
  
  lcd.setCursor(0,3);
  lcd.print(heaterC.Setpoint);
  lcd.setCursor(10,3);
  lcd.print(heaterC.Input);
}

float convert2dia(float in) {
  //converts an ADC reading to diameter
  //Inspired by Sprinter / Marlin thermistor reading
  byte numtemps = 5;
  const float table[numtemps][2] = {
    //{ADC reading in, diameter out}

    //REPLACE THESE WITH YOUR OWN READINGS AND DRILL BIT DIAMETERS
    
    { 0  , 3 },  // safety
    { 580  , 2.00 }, //2mm drill bit
    { 647  , 1.50 }, //1.5mm
    { 711  , 1.27 }, //1.27mm
    // { 1000  , 1 }, // 1mm
    { 1023  , 0 } //safety
  };
  byte i;
  float out;
  for (i = 1; i < numtemps; i++)
  {
    //check if we've found the appropriate row
    if (table[i][0] > in)
    {
      float slope = ((float)table[i][1] - table[i - 1][1]) / ((float)table[i][0] - table[i - 1][0]);
      float indiff = ((float)in - table[i - 1][0]);
      float outdiff = slope * indiff;
      float out = outdiff + table[i - 1][1];
      return (out);
      break;
    }
  }
}

//turns on firing pulse for heater 1
ISR(TIMER4_COMPA_vect){
  if(zero_cross){
    PORTA |= B00000001;    
  }
}

//turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect){
  if(zero_cross){
    PORTA |= B00000010;
  }
}

//turns on firing pulse for heater 3
ISR(TIMER4_COMPC_vect){
  if(zero_cross){
    PORTA |= B00000100;
  }
}

//turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect){
  PORTA &= !B00000111; 
  zero_cross = false;

#if TEST
  reset_timer();
#endif
}



void loop() {

// //menu function
//   bool menu_screenupdate = true;
//   if(menu_screenupdate){
//     lcd.clear();
    
//   }
//end of menu function


currentMillis = millis();
if(currentMillis - previousMillis >= Delay_readtemp){
  heater_loop();
};

// if(currentMillis - previousMillis >= Delay_readtemp){
  // if(zero_cross){
  // Serial.println("A");    
  // };
// }
}
