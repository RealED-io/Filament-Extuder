#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include "ACPID.h"


#ifndef LOGGING
  #define LOGGING 1
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
// or separate every debug

//pin declarations
#define HALL_SENSOR_PIN A8
// #define MOTOR_STEP A15
const uint8_t MOTOR_STEP = 5;
const uint8_t MOTOR_PULSE = 22;
const uint8_t zero_cross_pin = 18;    //zero cross pin for hardware interrupt
const uint8_t SPI_CLOCK = 52;        //for thermocouples
const uint8_t SPI_MISO = 50;
const uint8_t SPI_thermoA = 30;      
const uint8_t SPI_thermoB = 32;
const uint8_t SPI_thermoC = 34;
const uint8_t ROTARY_BUTTON = 19;
const uint8_t ROTARY_1A = 2;
const uint8_t ROTARY_1B = 3;
const uint8_t TACHO = 36;
// const uint8_t STEP = 1;
// PULSE PIN 28, 26, 24


//constants
const int pulse_reset_delay = 16666;
const int pulse_delay_max = 16660;
const int pulse_delay_min = 60;
const int motor_pulse_delay_max = 65535;
const int motor_pulse_delay_min = 500;
bool zero_cross = false;
const unsigned int Delay_readtemp = 250;
const unsigned int Delay_puller = 250;
const unsigned int Delay_display = 1000;
const unsigned int Delay_logging = 1000;
const unsigned int tacho_timeout = 30000;
unsigned long currentMillis = 0;
unsigned long previousMillis_display = 0;
unsigned long previousMillis_temp = 0;
unsigned long previousMillis_puller = 0;
unsigned long previousMillis_logging = 0;
unsigned long motor_pulsecount = 0;
bool read_loop = false;
bool isButton = false;
bool display_static = true;
bool display_dynamic = true;
bool display_valuesetter = false;
bool hazard = false;
bool control_RPM = true;
bool motor_run = false;
bool start_stop = false;
bool TEST_MODE = false;
bool SERIAL_LOGGING = false;
uint8_t selectorButton = 0;
static int oldposition;
uint8_t menulevel[4] = {0, 0, 0, 0};

//classes init
//set_temp, kP, kI, kD, reversed direction
ACPID heaterA(0, 450, 20, 5, REVERSE);    
ACPID heaterB(0, 450, 20, 5, REVERSE);           
ACPID heaterC(0, 450, 20, 5, REVERSE);         

MAX6675 thermoA(SPI_CLOCK, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_CLOCK, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_CLOCK, SPI_thermoC, SPI_MISO);

RotaryEncoder *encoder = nullptr;
LiquidCrystal_I2C lcd(0x27, 20, 4);

ACPID puller(1.75, 4500, 200, 50, REVERSE);

//function declarations
void safety_check();    // to be added
void thermo_check(double);
void AC_MOTOR_RUN();
void MOTOR_RUN();
void START_STOP();
void reset_timer();
void heater_loop();
void puller_loop();
float convert2dia(float);
float read_RPM();
void checkPosition();
void buttonPressed();
void check_mark(bool, uint8_t);
void cursor(uint8_t, uint8_t);
int8_t selector(int8_t);
void menuleveler();
void display_MainMenu();
void display_Menu_2();
void display_Menu_3();
void display_Menu_4();
void display_Menu_2_2();
void display_Calibrate_sizer();
void display_Set_heaterA();
void display_Set_heaterB();
void display_Set_heaterC();
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

  //sets PA to PA (pin ) as output
  DDRA |= B01010100;

  // sets timer 3 - for puller
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30);   // fast PWM, OCR4A TOP, prescaler 8
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
  TIMSK3 |= B00000010;  //enable compare match 3A

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

  // PWM for motor
  OCR3A = 62000;

  //firing delays rising edge
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  //firing delays falling edge
  OCR5A = pulse_reset_delay;

  sei();  //continue interrupts

  pinMode(zero_cross_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_cross_pin), reset_timer, RISING);
  encoder = new RotaryEncoder(ROTARY_1A, ROTARY_1B, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), buttonPressed, FALLING);    // reads button after release
  attachInterrupt(digitalPinToInterrupt(ROTARY_1A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_1B), checkPosition, CHANGE);

  //PID settings
  heaterA.Range(pulse_delay_min, pulse_delay_max);
  heaterB.Range(pulse_delay_min, pulse_delay_max);
  heaterC.Range(pulse_delay_min, pulse_delay_max);
  heaterA.PID_I_disableatError = 30;
  heaterB.PID_I_disableatError = 30;
  heaterC.PID_I_disableatError = 30;

  puller.Range(motor_pulse_delay_min, motor_pulse_delay_max);

  // hall sensor
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(TACHO, INPUT_PULLUP);
  pinMode(MOTOR_STEP, OUTPUT);
  pinMode(22, OUTPUT);

  // end lcd startup
  lcd.clear();
  cursor(2,6);
  lcd.print("FILAMENT");
  cursor(3,6);
  lcd.print("EXTRUDER");
  delay(3000);
  lcd.clear();

  MOTOR_RUN();
}





void loop() {
  // if (hazard)
  // {
  //   // STOP ALL
  // }
  currentMillis = millis();

  menuleveler();
  display_lcd();
  read_RPM();

  if (currentMillis - previousMillis_display >= Delay_display)
  {
    previousMillis_display = currentMillis;
    display_dynamic = true;
  }

  if (currentMillis - previousMillis_temp >= Delay_readtemp)
  {
    previousMillis_temp = currentMillis;
    heater_loop();
  }

  if (currentMillis - previousMillis_puller >= Delay_puller)
  {
    previousMillis_puller = currentMillis;
    puller_loop();
  }

  if (SERIAL_LOGGING)
  {
    if(currentMillis - previousMillis_logging >= Delay_logging)
    {
      previousMillis_logging = currentMillis;
      Serial.print(heaterA.Input);
      Serial.print(", ");
      Serial.print(heaterB.Input);
      Serial.print(", ");
      Serial.print(heaterC.Input);
      Serial.print(", ");
      Serial.println();

    }
  }
  



  // if (PID max based on ACPID)
  // {
  //   start time counter
  //   if (temp > 50 at given time)
  //   {
  //     stop the machine, thermocouple disconnected
  //   } 
  // }

  // add bool hazard = false
}


// function definitions
void reset_timer(){
  PORTA &= !B01010100; 
  zero_cross = true;
  TCNT4 = 0;
  TCNT5 = 0;
}

void heater_loop(){
  heaterA.Input = thermoA.readCelsius();    //store temp reading to ACPID.Input
  heaterB.Input = thermoB.readCelsius();
  heaterC.Input = thermoC.readCelsius();


  if (start_stop)
  {
    heaterA.Compute(Delay_readtemp);
    heaterB.Compute(Delay_readtemp);
    heaterC.Compute(Delay_readtemp);

    if (heaterA.Input > 0 || TEST_MODE)               // check if thermocouple disconnects
    {
      OCR4A = heaterA.Pulse_Delay;
    }
    else
    {
      OCR4A = 65535;                                  // turn heater off if thermocouple is off          
    }

    if (heaterB.Input > 0 || TEST_MODE)               
    {
      OCR4B = heaterB.Pulse_Delay;
    }
    else
    {
      OCR4B = 65535;                              
    }
    
    if (heaterC.Input > 0 || TEST_MODE)               
    {
      OCR4C = heaterC.Pulse_Delay;
    }
    else
    {
      OCR4C = 65535;                          
    }
  }

  #if LOGGING >= 5 //DEBUG 5
    // Serial.println(OCR4A);
    // Serial.println(OCR4B);
    // Serial.println(OCR4C);
  #endif
}

void puller_loop()
{
  puller.Input = convert2dia(analogRead(HALL_SENSOR_PIN));
  if (motor_run)
  {
    puller.Compute(Delay_puller);
    OCR3A = puller.Pulse_Delay;
  }
  
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

float read_RPM()
{
  static bool last_tacho;
  static unsigned long previousTacho;
  static unsigned long currentTacho;
  static float Tachotime;
  currentTacho = millis();
  if (previousTacho - currentTacho > tacho_timeout)
  {
    return NAN;
  }
  if (digitalRead(TACHO) != last_tacho)
  {
    Tachotime = currentTacho - previousTacho;
    previousTacho = currentTacho;
    last_tacho = !last_tacho;
  }
  return 60000 / (2*Tachotime);    // 1000 ms/s * 60 s  / ms
}

void MOTOR_RUN()
{
  if (motor_run)
  {
    digitalWrite(MOTOR_PULSE, HIGH);
    noInterrupts();
    // TCCR3A |= B01000000;
    // TIMSK3 |= B00000010;  //enable compare match 4A, 4B, 4C for heaters
    // OCR3A = 16000;
    TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30);   // fast PWM, OCR4A TOP, prescaler 8
    TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
    TIMSK3 |= B00000010;  //enable compare match 3A
    interrupts();

  }
  else
  {
    noInterrupts();
    TCCR3A = 0;
    TCCR3B = 0;
    // TCCR3A &= !B01000000;
    TIMSK3 &= !B00000010;  //disable compare match 4A, 4B, 4C for heaters   
    // OCR3A = 16000; 
    // // puller.PID_I = 65535;
    interrupts();
    digitalWrite(MOTOR_PULSE, LOW);
  }
}

void START_STOP()
{
  if (start_stop)
  {
    TIMSK4 |= B00001110;  //enable compare match 4A, 4B, 4C for heaters
  }
  else
  {
    TIMSK4 &= !B00001110;  //disable compare match 4A, 4B, 4C for heaters    
    heaterA.PID_I = pulse_delay_max;
    heaterA.PID_I = pulse_delay_max;
    heaterA.PID_I = pulse_delay_max;
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
      Serial.println(display_valuesetter);   
    #endif

  }
  last_interrupt_time = interrupt_time;
}

void check_mark(bool check, uint8_t select)
{
  if (check)
  {
    cursor(select,17);
    lcd.print("[/]");
  }
  else
  {
    cursor(select,17);
    lcd.print("[ ]");
  }
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
int8_t selector(int8_t numberofselection = 0)
{
  // static int oldposition; // made this to global variable
  if (numberofselection == 0)
  {
    return oldposition;
  }
  else
  {
    int position = encoder->getPosition() + 1;

    if ((oldposition != position) || display_static)
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
      
      if (display_valuesetter)
      {
        display_valuesetter = false;
        // Serial.println("nani");
      }
      else
      {
        display_valuesetter = true;
      }
      

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
        for(uint8_t i = 0; i < 4; i++)
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
  selector(4);
  // run only once to save processing time
  if (display_static)
  {
    cursor(1, 1);
    lcd.print("back");
    cursor(2, 1);
    lcd.print("Test mode");
    cursor(3, 1);
    lcd.print("RPM Control");
    cursor(4,1);
    lcd.print("Serial logging");

    check_mark(TEST_MODE, 2);
    check_mark(control_RPM, 3);
    check_mark(SERIAL_LOGGING, 4);

    display_static = false;
  } 
}

void display_Menu_2_2()
{
  selector(4);
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
    // set temps
    cursor(2, 4);
    lcd.print(heaterA.Setpoint);
    cursor(3, 4);
    lcd.print(heaterB.Setpoint);
    cursor(4, 4);
    lcd.print(heaterC.Setpoint);   
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
    if (heaterA.Input < 800) lcd.print(heaterA.Input);
    lcd.print(" ");
    cursor(3, 4);
    if (heaterB.Input < 800) lcd.print(heaterB.Input);
    lcd.print(" ");
    cursor(4, 4);
    if (heaterC.Input < 800) lcd.print(heaterC.Input);
    lcd.print(" ");
    cursor(6, 6);
    lcd.print(read_RPM());
    cursor(7, 6);
    // lcd.print(analogRead(HALL_SENSOR_PIN));
    lcd.print(convert2dia(analogRead(HALL_SENSOR_PIN)));
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
    if (motor_run)
    {
      lcd.print("Motor ON");
    }
    else
    {
      lcd.print("Motor OFF");      
    }
    cursor(6, 1);
    lcd.print("RPM");
    cursor(7, 1);
    lcd.print("Size");
    cursor(8, 1);
    if (start_stop)
    {
      lcd.print("Heatr ON");
    }
    else
    {
      lcd.print("Heatr OFF");      
    }
    display_static = false;
  }
}

void display_Calibrate_sizer()
{

}

void display_Set_heaterA()
{ 
  if (display_dynamic)
  {
    cursor(2, 4);
    lcd.print(heaterA.Setpoint);
    lcd.print(" ");
  }
  
  if (display_static)
  {
    cursor(2, 1);
    lcd.print("T1");
    encoder->getDirection();    // resets value of direction before using it to set
  }
  heaterA.Setpoint += int(encoder->getDirection());
  heaterA.PID_I = pulse_delay_max;
  display_static = false;
}

void display_Set_heaterB()
{ 
  if (display_dynamic)
  {
    cursor(3, 4);
    lcd.print(heaterB.Setpoint);
    lcd.print(" ");
  }
  
  if (display_static)
  {
    cursor(3, 1);
    lcd.print("T2");
    encoder->getDirection();    // resets value of direction before using it to set
  }
  heaterB.Setpoint += int(encoder->getDirection());
  heaterB.PID_I = pulse_delay_max;
  display_static = false;
}

void display_Set_heaterC()
{ 
  if (display_dynamic)
  {
    cursor(4, 4);
    lcd.print(heaterC.Setpoint);
    lcd.print(" ");
  }
  
  if (display_static)
  {
    cursor(4, 1);
    lcd.print("T3");
    encoder->getDirection();    // resets value of direction before using it to set
  }
  heaterC.Setpoint += int(encoder->getDirection());
  heaterC.PID_I = pulse_delay_max;
  display_static = false;
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

      case 2:                            // extrude/set temp/T1
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(1);
          break;
        }
        display_Set_heaterA();
        break;

      case 3:                            // extrude/set temp/T2
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(2);
          break;
        }
        display_Set_heaterB();
        break;
      
      case 4:                            // extrude/set temp/T3
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(3);
          break;
        }
        display_Set_heaterC();
        break;     


      default:
        menulevel[2] = 0;
        break;
      }
      break;

    case 3:                          // extrude/start
      switch (menulevel[2])
      {
      case 0:                            // extrude/start/
        display_Menu_2_3();
        break;
      
      case 1:                            // extrude/start/back
        menulevel[1] = 0;
        menulevel[2] = 0;
        display_Menu_2();
        break;

      case 2:                            // extrude/start/T1
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(1);
          break;
        }
        display_Set_heaterA();
        break;

      case 3:                            // extrude/start/T2
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(2);
          break;
        }
        display_Set_heaterB();
        break;
      
      case 4:                            // extrude/start/T3
        if (!display_valuesetter)
        {
          menulevel[2] = 0;
          encoder->setPosition(3);
          break;
        }
        display_Set_heaterC();
        break; 
      
      case 5:                            // extrude/start/run motor
        motor_run = !motor_run;
        MOTOR_RUN();
        // menulevel[1] = 0;
        display_static = true;
        menulevel[2] = 0;
        break;

      case 8:                            // extrude/start/start htr
        start_stop = !start_stop;
        START_STOP();
        // menulevel[1] = 0;
        display_static = true;
        menulevel[2] = 0;
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
    
    case 2:                             // settings/Test mode
      TEST_MODE = !TEST_MODE;
      display_static = true;
      menulevel[1] = 0;
      break;

    case 3:                             // settings/control RPM
      control_RPM = !control_RPM;
      display_static = true;
      menulevel[1] = 0;
      break;

    case 4:                             // settings/serial logging
      SERIAL_LOGGING = !SERIAL_LOGGING;
      display_static = true;
      menulevel[1] = 0;
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

// unnecessary code
ISR(TIMER3_COMPA_vect)
{
  // motor_pulsecount++;
}

// turns on firing pulse for heater 1
ISR(TIMER4_COMPA_vect)
{
  if (zero_cross)
  {
    PORTA |= B01000000; // turns pin 28 on
  }
}

// turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect)
{
  if (zero_cross)
  {
    PORTA |= B00010000; // turns pin 26 on
  }
}

// turns on firing pulse for heater 3
ISR(TIMER4_COMPC_vect)
{
  if (zero_cross)
  {
    PORTA |= B00000100; // turns pin 24 on
  }
}

// turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect)
{
  PORTA &= !B01010100; // turns pin 28, 26, 24 off
  zero_cross = false;

  if (TEST_MODE)
  {
    reset_timer();
  }
}