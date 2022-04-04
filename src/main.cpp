#include <Arduino.h>


const int pulse_delay_max = 16600;

int pulse_delay_1 = 0;
int pulse_delay_2 = 0;
int pulse_delay_3 = 0;


void setup() {
  cli(); //stops interrupts


  //sets PA0 to PA2 (pin 22-24) as output
  DDRA |= B00000111;

  //sets timer 3
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= B00000010;  //set prescaler to 8
  TIMSK3 |= B00000110;  //enable compare match 3A, 3B

  //sets timer 4
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= B00000010;  //set prescaler to 8
  TIMSK4 |= B00000110;  //enable compare match 4A, 4B

  //sets timer 5
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= B00000010;  //set prescaler to 8
  TIMSK5 |= B00000110;  //enable compare match 5A, 5B

  //firing delays rising edge
  OCR3A = pulse_delay_1;
  OCR4A = pulse_delay_2;
  OCR5A = pulse_delay_3;

  //firing delays falling edge
  OCR3B = pulse_delay_max;
  OCR4B = pulse_delay_max;
  OCR5B = pulse_delay_max;
  

  sei();  //continue interrupts
}

//firing pulse for heater 1
//turns on firing pulse for heater 1
ISR(TIMER3_COMPA_vect){
  PORTA |= B00000001;
}
//turns off firing pulse for heater 1
ISR(TIMER3_COMPB_vect){
  PORTA &= !B00000001;                //**********can have all the pins falling edge (PORTA &= !B00000111) test changes
}

//firing pulse for heater 2
//turns on firing pulse for heater 2
ISR(TIMER4_COMPA_vect){
  PORTA |= B00000010;
}
//turns off firing pulse for heater 2
ISR(TIMER4_COMPB_vect){
  PORTA &= !B00000010;
}

//firing pulse for heater 3
//turns on firing pulse for heater 3
ISR(TIMER5_COMPA_vect){
  PORTA |= B00000100;
}
//turns off firing pulse for heater 3
ISR(TIMER5_COMPB_vect){
  PORTA &= !B00000100;
}



void loop() {
  // put your main code here, to run repeatedly:
}