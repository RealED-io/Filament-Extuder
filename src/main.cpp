#include <Arduino.h>


const int pulse_delay_max = 16600;

int pulse_delay_1 = 0;
int pulse_delay_2 = 0;
int pulse_delay_3 = 0;


void setup() {
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
  OCR4A = pulse_delay_1;
  OCR4B = pulse_delay_2;
  OCR4C = pulse_delay_3;

  //firing delays falling edge
  OCR5A = pulse_delay_max;
  

  sei();  //continue interrupts
}


//turns on firing pulse for heater 1
ISR(TIMER4_COMPA_vect){
  PORTA |= B00000001;
}
//turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect){
  PORTA |= B00000010;
}
//turns on firing pulse for heater 3
ISR(TIMER4_COMPB_vect){
  PORTA |= B00000100;
}

//turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect){
  PORTA &= !B00000111; 
}



void loop() {
  // put your main code here, to run repeatedly:
}