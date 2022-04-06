#include <Arduino.h>
#include <PID_v1.h>
#include <max6675.h>
#include "ACHeater.h"

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
const int readtempDelay = 500;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

//classes init
ACHeater heaterA(0,0,0,           //temp, pulse delay, set
                  1,1,1);         //kP, kI, kD
ACHeater heaterB(0,0,0,           //temp, pulse delay, set
                  1,1,1);         //kP, kI, kD
ACHeater heaterC(0,0,0,           //temp, pulse delay, set
                  1,1,1);         //kP, kI, kD //***********************************add thermocouple pins to the class

PID PID_heaterA(&heaterA.Temp, &heaterA.Pulse_Delay, &heaterA.Set_Temp,
                heaterA.kP, heaterA.kI, heaterA.kD, REVERSE);
PID PID_heaterB(&heaterB.Temp, &heaterB.Pulse_Delay, &heaterB.Set_Temp,
                heaterB.kP, heaterB.kI, heaterB.kD, REVERSE);
PID PID_heaterC(&heaterC.Temp, &heaterC.Pulse_Delay, &heaterC.Set_Temp,
                heaterC.kP, heaterC.kI, heaterC.kD, REVERSE);

MAX6675 thermoA(SPI_clock, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_clock, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_clock, SPI_thermoC, SPI_MISO);


//function declarations
void reset_timer();

//*************** add void PID_compute_routine();


void setup() {
  // Serial.begin(9600);   //for testing
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
  
  attachInterrupt(zero_cross_pin, reset_timer, RISING);

  sei();  //continue interrupts
}

void reset_timer(){
  zero_cross = true;
  TCNT4 = 0;
  TCNT5 = 0;
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
  currentMillis = millis();
  if(currentMillis - previousMillis >= readtempDelay){
    previousMillis += readtempDelay;

    //*************insert PID compute loop here
    

  }
    
    



  //for testing
//   if (Serial.available() > 0)
//  {
//     //Create a place to hold the incoming message
//     static char message[MAX_MESSAGE_LENGTH];
//     static unsigned int message_pos = 0;

//     //Read the next available byte in the serial receive buffer
//     char inByte = Serial.read();

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

//       Serial.println(message);
      
//       if(message[0] == 'a'){
//         Serial.println("case a");
//         OCR4A = number;

//       }else if (message[0] == 'b')
//       {
//         Serial.println("case b");
//         OCR4B = number;
//       }else if (message[0] == 'c')
//       {
//         Serial.println("case c");
//         OCR4C = number;
//       }else
//       {
//         number = atoi(message);
//         Serial.println(number);
//       }
      

//       //Reset for the next message
//       message_pos = 0;
//    }
//  }
}