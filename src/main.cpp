#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include "ACHeater.h"

#if false
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

// //for testing
// const unsigned int MAX_MESSAGE_LENGTH = 6;
// unsigned int number = 0;

//pin declarations
const int zero_cross_pin = 3;    //zero cross pin for hardware interrupt
const int SPI_clock = 52;
const int SPI_MISO = 50;
const int SPI_thermoA = 10;
const int SPI_thermoB = 11;
const int SPI_thermoC = 12;


//constants
const int pulse_delay_max = 16600;
bool zero_cross = false;
const int Delay_readtemp = 500;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
bool read_loop = false;

//classes init
//set_temp, kP, kI, kD, reversed direction
ACHeater heaterA(50, 10, 20, 20, true);    
ACHeater heaterB(50, 10, 20, 20, true);           
ACHeater heaterC(50, 10, 20, 20, true);         
                           //***********************************add thermocouple pins to the class

// double ATime = 0, BTime = 0, CTime = 0;
// double Aerror_previous = 0, Berror_previous = 0, Cerror_previous = 0;
// double APID_I, BPID_I, CPID_I;

// PID PID_heaterA(&heaterA.Temp, &heaterA.Pulse_Delay, &heaterA.Set_Temp,
//                 heaterA.kP, heaterA.kI, heaterA.kD, REVERSE);
// PID PID_heaterB(&heaterB.Temp, &heaterB.Pulse_Delay, &heaterB.Set_Temp,
//                 heaterB.kP, heaterB.kI, heaterB.kD, REVERSE);
// PID PID_heaterC(&heaterC.Temp, &heaterC.Pulse_Delay, &heaterC.Set_Temp,
//                 heaterC.kP, heaterC.kI, heaterC.kD, REVERSE);

MAX6675 thermoA(SPI_clock, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_clock, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_clock, SPI_thermoC, SPI_MISO);

LiquidCrystal_I2C lcd(0x27,20,4);


//function declarations
void reset_timer();
// double PID_compute();
//*************** add void PID_compute_routine();


void setup() {
  Debugbegin(9600);   //for testing
  
  cli(); //stops interrupts

  //sets PA0 to PA2 (pin 22-24) as output
  DDRA |= B00000111;

  // //sets timer 3
  // TCCR3A = 0;
  // TCCR3B = 0;
  // TCCR3B |= B00000100;  //set prescaler to 256
  // TIMSK3 |= B00000010;  //enable compare match 3A for timed readings (to remove millis() )

  // OCR3A = 31250; //0.5 secs

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

  attachInterrupt(zero_cross_pin, reset_timer, RISING);

  //lcd initialization
  lcd.init();
  lcd.backlight();
  
  //PID settings
  // PID_heaterA.SetOutputLimits(1, pulse_delay_max);
  // PID_heaterB.SetOutputLimits(1, pulse_delay_max);
  // PID_heaterC.SetOutputLimits(1, pulse_delay_max);
  heaterA.Range(1,pulse_delay_max);
  heaterB.Range(1,pulse_delay_max);
  heaterC.Range(1,pulse_delay_max);
}

void reset_timer(){
  zero_cross = true;
  TCNT4 = 0;
  TCNT5 = 0;
}

// void PID_compute(double kP, double kI, double kD, double Temp, double Set_Temp, int heaternumber){
//   double PID_P, PID_I, PID_D, PID_value, error, error_previous;
//   double Time, Time_elapsed, Time_previous;
//   // Debugprintln(kP);
//   // Debugprintln(kI);
//   // Debugprintln(kD);
//   error = Set_Temp - Temp;
//   // Debugprintln(error);
//   if(error > 30){
//     PID_I = 0;
//   }

//   PID_P = kP * error;

//   if(heaternumber == 1){
//     Time = ATime;
//     error_previous = Aerror_previous;
//     PID_I = APID_I;
//   }else if (heaternumber == 2){
//     Time = BTime;
//     error_previous = Berror_previous;
//     PID_I = BPID_I;
//   }else if (heaternumber == 3){
//     Time = CTime;
//     error_previous = Cerror_previous;
//     PID_I = CPID_I;
//   }
//   PID_I = PID_I + (kI * error);

//   Time_previous = Time;
//   Time = millis();

//   Time_elapsed = (Time - Time_previous) / 1000;

//   PID_D = kD * (error - error_previous) / Time_elapsed;

//   // Debugprintln(PID_P);
//   // Debugprintln(PID_I);
//   // Debugprintln(PID_D);
//   PID_value = PID_P + PID_I + PID_D;

  
//   PID_value = 16601 - PID_value;
  
//   if(PID_value < 1){
//     PID_value = 1;
//   }
//   if(PID_value > 16600){
//     PID_value = 16600;
//   }

//   if(heaternumber == 1){
//     ATime = Time;
//     Aerror_previous = error;
//     heaterA.Pulse_Delay = int(PID_value);
//     APID_I = PID_I;
//   }else if (heaternumber == 2){
//     BTime = Time;
//     Berror_previous = error;
//     heaterB.Pulse_Delay = int(PID_value);
//     BPID_I = PID_I;
//   }else if (heaternumber == 3){
//     CTime = Time;
//     Cerror_previous = error;
//     heaterC.Pulse_Delay = int(PID_value);
//     CPID_I = PID_I;
//   }
// }

//for timed temp readings (to remove millis() )
// ISR(TIMER3_COMPA_vect){
//   read_loop = true;
// }

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

  //for testing
  reset_timer();
}



void loop() {
  // PID_heaterA.Compute();
  // PID_heaterB.Compute();
  // PID_heaterC.Compute();



  currentMillis = millis();
  if(currentMillis - previousMillis >= Delay_readtemp){
    previousMillis += Delay_readtemp;

    heaterA.Temp = thermoA.readCelsius();
    heaterB.Temp = thermoB.readCelsius();
    heaterC.Temp = thermoC.readCelsius();
    
    // Debugprintln(heaterA.kP);
    // Debugprintln(heaterB.Temp);
    // Debugprintln(heaterC.Set_Temp);
    // PID_compute(heaterA.kP, heaterA.kI, heaterA.kD, heaterA.Temp, heaterA.Set_Temp, 1);
    // PID_compute(heaterB.kP, heaterB.kI, heaterB.kD, heaterB.Temp, heaterB.Set_Temp, 2);
    // PID_compute(heaterC.kP, heaterC.kI, heaterC.kD, heaterC.Temp, heaterC.Set_Temp, 3);
    //PID compute here or in the loop

    heaterA.Compute(Delay_readtemp);
    heaterB.Compute(Delay_readtemp);
    heaterC.Compute(Delay_readtemp);

    OCR4A = heaterA.Pulse_Delay;
    OCR4B = heaterB.Pulse_Delay;
    OCR4C = heaterC.Pulse_Delay;

    Debugprintln(heaterA.Pulse_Delay);
    Debugprintln(OCR4B);
    Debugprintln(OCR4C);

    // lcd.clear();
    
    // lcd.setCursor(0,0);
    // lcd.print("Set: ");
    // lcd.setCursor(10,0);
    // lcd.print("Real: ");

    // lcd.setCursor(0,1);
    // lcd.print(heaterA.Set_Temp);
    // lcd.setCursor(10,1);
    // lcd.print(heaterA.Temp);

    // lcd.setCursor(0,2);
    // lcd.print(heaterB.Set_Temp);
    // lcd.setCursor(10,2);
    // lcd.print(heaterB.Temp);
    
    // lcd.setCursor(0,3);
    // lcd.print(heaterC.Set_Temp);
    // lcd.setCursor(10,3);
    // lcd.print(heaterC.Temp);

    // read_loop = false;
  }
    
    



  //for testing
//   if (Debugavailable() > 0)
//  {
//     //Create a place to hold the incoming message
//     static char message[MAX_MESSAGE_LENGTH];
//     static unsigned int message_pos = 0;

//     //Read the next available byte in the Debug receive buffer
//     char inByte = Debug.read();

//     //Message coming in (check not terminating character) and guard for over message size
//     if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
//     {
//       //Add the incoming byte to our message
//       message[message_pos] = inByte;
//       message_pos++;
//     }

//     //Full message received...
//     else
//     {
//       //Add null character to string
//       message[message_pos] = '\0';

//       Debugprintln(message);
      
//       if(message[0] == 'a'){
//         Debugprintln("case a");
//         OCR4A = number;

//       }else if (message[0] == 'b')
//       {
//         Debugprintln("case b");
//         OCR4B = number;
//       }else if (message[0] == 'c')
//       {
//         Debugprintln("case c");
//         OCR4C = number;
//       }else
//       {
//         number = atoi(message);
//         Debugprintln(number);
//       }
      

//       //Reset for the next message
//       message_pos = 0;
//    }
//  }
}