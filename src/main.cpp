#include <Arduino.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include "ACPID.h"

#if true
  #define Debugbegin(x) Serial.begin(x)
  #define Debugprint(x) Serial.print(x)
  #define Debugprintln(x) Serial.println(x)
  #define Debugavailable(x) Serial.available(x)
#else
  #define Debugbegin(x)
  #define Debugprint(x)
  #define Debugprintln(x)
  #define Debugavailable(x)
#endif

//************************ add ArduinoLog and remove PID library


//pin declarations
const int zero_cross_pin = 3;    //zero cross pin for hardware interrupt
const int SPI_clock = 52;        //for thermocouples
const int SPI_MISO = 50;
const int SPI_thermoA = 10;      
const int SPI_thermoB = 11;
const int SPI_thermoC = 12;
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
ACPID heaterA(53, 10, 10, 10, true);    
ACPID heaterB(50, 10, 10, 10, true);           
ACPID heaterC(50, 10, 10, 10, true);         

MAX6675 thermoA(SPI_clock, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_clock, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_clock, SPI_thermoC, SPI_MISO);

LiquidCrystal_I2C lcd(0x27,20,4);


//function declarations
void reset_timer();
void heater_loop();

//*************** add void PID_compute_routine();


void setup() {
  Debugbegin(9600);   //for testing
  
  
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
  heaterA.Range(600,16000);
  heaterB.Range(600,16000);
  heaterC.Range(600,16000);
}

void reset_timer(){
  PORTA &= !B00000111; 
  zero_cross = true;
  TCNT4 = 0;
  TCNT5 = 0;
}

void heater_loop(){
  previousMillis += Delay_readtemp;

  heaterA.Input = thermoA.readCelsius();    //store temp reading to ACPID.Input
  heaterB.Input = thermoB.readCelsius();
  heaterC.Input = thermoC.readCelsius();

  heaterA.Compute(Delay_readtemp);
  heaterB.Compute(Delay_readtemp);
  heaterC.Compute(Delay_readtemp);

  OCR4A = heaterA.Pulse_Delay;
  OCR4B = heaterB.Pulse_Delay;
  OCR4C = heaterC.Pulse_Delay;

  // Debugprintln(OCR4A);
  // Debugprintln(OCR4B);
  // Debugprintln(OCR4C);

  // lcd.clear();
  
  // lcd.setCursor(0,0);
  // lcd.print("Set: ");
  // lcd.setCursor(10,0);
  // lcd.print("Real: ");

  // lcd.setCursor(0,1);
  // lcd.print(heaterA.Setpoint);
  // lcd.setCursor(10,1);
  // lcd.print(heaterA.Input);

  // lcd.setCursor(0,2);
  // lcd.print(heaterB.Setpoint);
  // lcd.setCursor(10,2);
  // lcd.print(heaterB.Input);
  
  // lcd.setCursor(0,3);
  // lcd.print(heaterC.Setpoint);
  // lcd.setCursor(10,3);
  // lcd.print(heaterC.Input);
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

  // //for testing
  // reset_timer();
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
