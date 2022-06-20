#include <Arduino.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
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
const uint8_t SPI_thermoA = 44;      
const uint8_t SPI_thermoB = 46;
const uint8_t SPI_thermoC = 48;
const uint8_t ROTARY_BUTTON = 2;
const uint8_t ROTARY_1A = 18;
const uint8_t ROTARY_1B = 19;
// PULSE PIN 22, 23, 24


//constants
const int pulse_delay_max = 16600;
bool zero_cross = false;
const int Delay_readtemp = 500;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
bool read_loop = false;
bool isButton = false;
bool display_static = true;
bool display_dynamic = true;
uint8_t selectorButton = 0;
static int oldposition;
uint8_t menulevel[4] = {0, 0, 0, 0};

//classes init
//set_temp, kP, kI, kD, reversed direction
ACPID heaterA(90, 2, 2, 2, true);    
ACPID heaterB(90, 2, 2, 2, true);           
ACPID heaterC(90, 2, 2, 2, true);         

MAX6675 thermoA(SPI_clock, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_clock, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_clock, SPI_thermoC, SPI_MISO);

RotaryEncoder *encoder = nullptr;
LiquidCrystal_I2C lcd(0x27, 20, 4);


//function declarations
void reset_timer();
void heater_loop();
float convert2dia(float);
void checkPosition();
void buttonPressed();
void cursor(uint8_t, uint8_t);
int8_t selector(int);
void menuleveler();
void display_MainMenu();
void display_Menu_2();
void display_Menu_3();
void display_Menu_4();
void display_Menu_2_2();
void display_Menu_2_3();
void display_lcd();


void setup() {
  #if LOGGING > 0
  Serial.begin(9600);   //for testing
  #endif

  //lcd initialization
  lcd.init();
  lcd.backlight();
  lcd.print("please wait");

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
  encoder = new RotaryEncoder(ROTARY_1A, ROTARY_1B, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), buttonPressed, FALLING);    // reads button after release
  attachInterrupt(digitalPinToInterrupt(ROTARY_1A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_1B), checkPosition, CHANGE);

  //PID settings
  heaterA.Range(600,16660);
  heaterB.Range(600,16660);
  heaterC.Range(600,16660);
  heaterA.PID_I_disableatError = 5;
  heaterB.PID_I_disableatError = 5;
  heaterC.PID_I_disableatError = 5;

  // hall sensor
  pinMode(Hall_Sensor_Pin, INPUT);

  // end lcd startup
  lcd.clear();
  lcd.print("FILAMENT EXTRUDER");
  delay(1000);
  lcd.clear();
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
  display_dynamic = true;
};
  menuleveler();
  display_lcd();

// if(currentMillis - previousMillis >= Delay_readtemp){
  // if(zero_cross){
  // Serial.println("A");    
  // };
// }
}

// function definitions
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

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void buttonPressed()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    isButton = true;
    #if LOGGING >= 6
      Serial.println("x");    
    #endif

  }
  last_interrupt_time = interrupt_time;
}

void cursor(uint8_t swt, uint8_t offset = 0)
{
  switch (swt)
  {
  case 1:
    lcd.setCursor(0 + offset, 0);
    break;
  case 2:
    lcd.setCursor(0 + offset, 1);
    break;
  case 3:
    lcd.setCursor(0 + offset, 2);
    break;
  case 4:
    lcd.setCursor(0 + offset, 3);
    break;
  case 5:
    lcd.setCursor(10 + offset, 0);
    break;
  case 6:
    lcd.setCursor(10 + offset, 1);
    break;
  case 7:
    lcd.setCursor(10 + offset, 2);
    break;
  case 8:
    lcd.setCursor(10 + offset, 3);
    break;
  default:
#if LOGGING >= 5
    Serial.println("ERR: LCD setcursor");
#endif
    break;
  }
}

// when no argument is supplied, returns old position
int8_t selector(int numberofselection = 0)
{
  // static int oldposition; // made this to global variable
  if (numberofselection == 0)
  {
    return oldposition;
  }
  else
  {
    int position = encoder->getPosition() + 1;

    if (oldposition != position)
    {

      // erases old cursor
      cursor(oldposition);
      lcd.print(" ");

      // min selector position
      if (position <= 1)
      {
        cursor(1);
        encoder->setPosition(0);
        oldposition = 1;
      }
      // max selector position
      else if (position >= numberofselection)
      {
        cursor(numberofselection);
        encoder->setPosition(numberofselection - 1);
        oldposition = numberofselection;
      }
      // in between min max
      else
      {
        cursor(position);
        oldposition = position;
      }

      lcd.print(">");

#if LOGGING >= 6 // TRACE logging
      Serial.print(position);
      Serial.print(" = ");
      Serial.println(oldposition);
#endif
    }
  }
  return oldposition;
}


// LCD Displays functions
void menuleveler()
{
  if (isButton)
    {
      display_static = true;
      lcd.clear();
      // check the menu level
      if(menulevel[0] != 0){
        if(menulevel[1] != 0){
          if(menulevel[2] != 0){
            if(menulevel[3] != 0){
            }else{menulevel[3] = oldposition;}
          }else{menulevel[2] = oldposition;}
        }else{menulevel[1] = oldposition;}
      }else{menulevel[0] = oldposition;}
      //reset button state
      isButton = false;
      
      #if LOGGING >= 6
        for(uint8_t i = 0; i < 3; i++)
        {
          Serial.print(menulevel[i]);
          Serial.print(" ");
        }
        Serial.println();
      #endif
    }
}

void display_MainMenu()
{
  selector(4);
  //run every poll
  cursor(1, 1);
  lcd.print(oldposition);

  // run only once to save processing time
  if (display_static)
  {

    cursor(2, 1);
    lcd.print("Extrude");
    cursor(3, 1);
    lcd.print("Calibrate");
    cursor(4, 1);
    lcd.print("Settings");
    display_static = false;
  }
}

void display_Menu_2()
{
  selector(3);
  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("Set Temps");
    cursor(3, 1);
    lcd.print("Start");
    display_static = false;
  }
}

void display_Menu_3()
{
  selector(4);
  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("heater PID");
    cursor(3, 1);
    lcd.print("puller PID");
    cursor(4, 1);
    lcd.print("size sensor");
    display_static = false;
  }
}

void display_Menu_4()
{
  selector(2);
  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("timeout monitoring");
    display_static = false;
  }
}

void display_Menu_2_2()
{
  selector(4);
  //add heater set temps

  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("T1");
    cursor(3, 1);
    lcd.print("T2");
    cursor(4, 1);
    lcd.print("T3");    
    display_static = false;
  }
}

void display_Menu_2_3()
{
  selector(8);
  //add heater read temps
  if (display_dynamic)
  {
    cursor(2, 4);
    lcd.print(heaterA.Input);
    cursor(3, 4);
    lcd.print(heaterB.Input);
    cursor(4, 4);
    lcd.print(heaterC.Input);
    cursor(7, 6);
    lcd.print(convert2dia(analogRead(Hall_Sensor_Pin)));
    display_dynamic = false;
  }
  
  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("T1");
    cursor(3, 1);
    lcd.print("T2");
    cursor(4, 1);
    lcd.print("T3");
    cursor(5, 1);
    lcd.print("pause");
    cursor(6, 1);
    lcd.print("RPM");
    cursor(7, 1);
    lcd.print("Size");
    cursor(8, 1);
    lcd.print("stop");      
    display_static = false;
  }
}



void display_lcd()
{
  switch (menulevel[0])
  {
  case 0:                          // Main Menu
    display_static = true;
    display_MainMenu();
    break;

  case 2:                          // extrude
    switch (menulevel[1])
    {
    case 0:                           // extrude/
      display_Menu_2();
      break;

    case 1:                           // extrude/back
      menulevel[0] = 0;
      menulevel[1] = 0;
      display_MainMenu();
      break;

    case 2:                           // extrude/set temp
      switch (menulevel[2])
      {
      case 0:                            // extrude/set temp/
        display_Menu_2_2();
        break;
        
      case 1:                            // extrude/set temp/back
        menulevel[1] = 0;
        menulevel[2] = 0;
        display_Menu_2();
        break;

      default:
        menulevel[2] = 0;
        break;
      }
      break;

    case 3:                          // extrude/start
      switch (menulevel[2])
      {
      case 0:                           // extrude/start/
        display_Menu_2_3();
        break;
      
      case 1:                           // extrude/start/back
        menulevel[1] = 0;
        menulevel[2] = 0;
        display_Menu_2();
        break;

      default:
        menulevel[2] = 0;
        break;
      }
      break;

    default:
      menulevel[1] = 0;
      break;
    }
    break;

  case 3:                             // calibrate
    switch (menulevel[1])
    {
    case 0:
      display_Menu_3();
      break;
    
    case 1:                           // calibrate/back
      menulevel[0] = 0;
      menulevel[1] = 0;
      display_MainMenu();
      break;
    
    default:
      menulevel[1] = 0;
      break;
    }
    break;

  case 4:                            // settings
    switch (menulevel[1])
    {
    case 0:                             // settings/
      display_Menu_4();
      break;
    
    case 1:                             // settings/back
      menulevel[0] = 0;
      menulevel[1] = 0;
      display_MainMenu();
      break;

    default:
      menulevel[1] = 0;
      break;
    }
    break;

  default:
    menulevel[0] = 0;
    break;
  }
}

//turns on firing pulse for heater 1
ISR(TIMER4_COMPA_vect){
  if(zero_cross){
    PORTA |= B00000001;    // turns pin 22 on
  }
}

//turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect){
  if(zero_cross){
    PORTA |= B00000010;    // turns pin 23 on
  }
}

//turns on firing pulse for heater 3
ISR(TIMER4_COMPC_vect){
  if(zero_cross){
    PORTA |= B00000100;    // turns pin 24 on
  }
}

//turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect){
  PORTA &= !B00000111;     // turns pin 22, 23, 24 off
  zero_cross = false;

#if TEST
  reset_timer();
#endif
}