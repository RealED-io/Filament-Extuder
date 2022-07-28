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

// PIN DECLARATIONS
#define HALL_SENSOR_PIN A8
#define TACHO A9
const uint8_t STEPPER_PULSE = 5;				// stepper motor pulse pin (OCR3A)
const uint8_t MOTOR_PULSE = 31;					// AC motor pulse pin/or enable pin	
const uint8_t ZERO_CROSS_PIN = 18; 				// zero cross pin for hardware interrupt
const uint8_t SPI_CLOCK = 52;	   				// for thermocouples
const uint8_t SPI_MISO = 50;
const uint8_t SPI_thermoA = 30;
const uint8_t SPI_thermoB = 32;
const uint8_t SPI_thermoC = 34;
const uint8_t ROTARY_BUTTON = 19;
const uint8_t ROTARY_1A = 2;
const uint8_t ROTARY_1B = 3;
// const uint8_t TACHO = 36;
// const uint8_t STEP = 1;
// PULSE PIN 28, 26, 24

// changeable constants
const int pulse_delay_max = 16660;				// max phase control pulse delay in n / 2E6 secs for every zerocross
const int pulse_delay_min = 60;					// min phase control pulse delay in n / 2E6 secs for every zerocross
const int pulse_reset_delay = 16666;			// reset delay for phase control pulse in n / 2E6 secs for every zerocross
const int motor_pulse_delay_max = 35000;		// max stepper PWM pulsewidth in n / 2E6 secs
const int motor_pulse_delay_min = 10000;		// min stepper PWM pulsewidth in n / 2E6 secs
const unsigned int Delay_readtemp = 250;		// temperature reading delay in ms
const unsigned int Delay_puller = 250;			// puller speed updates delay in ms
const unsigned int Delay_display = 1000;		// display delay in ms
const unsigned int Delay_logging = 1000;		// serial logging delay in ms
const unsigned int Delay_read_dia = 25;			// infidel read delay in ms
const unsigned int tacho_timeout = 10000;		// tachometer reading timeout in ms


unsigned long currentMillis = 0;
unsigned long previousMillis_display = 0;
unsigned long previousMillis_temp = 0;
unsigned long previousMillis_puller = 0;
unsigned long previousMillis_logging = 0;
unsigned long previousMillis_read_dia = 0;
unsigned long motor_pulsecount = 0;
int analog_ave = 0;

bool zero_cross = false;
bool read_loop = false;
bool isButton = false;
bool display_static = true;
bool display_dynamic = true;
bool display_valuesetter = false;
bool hazard = false;
bool control_RPM = true;
bool motor_run = false;
bool stepper_run = false;
bool start_stop = false;
bool TEST_MODE = false;
bool SERIAL_LOGGING = false;
uint8_t selectorButton = 0;
static int oldposition;
uint8_t menulevel[5] = {0, 0, 0, 0, 0};
float RPM;
String RPM_string;
float stepRPM;
String stepRPM_string;
float dia_analog_val[6] = {};
float dia_size_cal[6] = {};
// float dia_analog_val[6] = {0, 600, 622, 692, 883, 1023};
// float dia_size_cal[6] = {3, 2.00, 1.75, 1.60, 1.20, 0};
uint8_t dia_analog_val_idx[6] = {68, 72, 76, 80, 84, 88};
uint8_t dia_size_cal_idx[6] = {92, 96, 100, 104, 108, 112};

// classes init
// // set_temp, index kP, kI, kD, reversed direction
// ACPID heaterA(0, 4, 8, 12, REVERSE);
// ACPID heaterB(0, 16, 20, 24, REVERSE);
// ACPID heaterC(0, 28, 32, 36, REVERSE);
ACPID heaterA(REVERSE);
ACPID heaterB(REVERSE);
ACPID heaterC(REVERSE);
// ACPID heaterC(0, 450, 20, 5, REVERSE); // original code

MAX6675 thermoA(SPI_CLOCK, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_CLOCK, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_CLOCK, SPI_thermoC, SPI_MISO);

RotaryEncoder *encoder = nullptr;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ACPID puller(1.9, 40, 44, 48, DIRECT);
ACPID puller(DIRECT);
// ACPID puller(1.9, 10, 10, 5, DIRECT);

// function declarations
void safety_check(); // to be added
void thermo_check(float);
void STEPPER_RUN();
void MOTOR_RUN();
void START_STOP();
void reset_timer();
void heater_loop();
void puller_loop();
void ANALOG_AVE(int, int *);
float convert2dia(float);
float read_RPM();
void checkPosition();
void buttonPressed();
void check_mark(bool, uint8_t);
void cursor(uint8_t, uint8_t);
int8_t selector(int8_t);
void menuleveler();
void display_Setter(float *, float, uint8_t, String);
// void display_Setter_kPID();
void display_lcd();
void save_set();
void load_set();
void save_cal(unsigned int, float, unsigned int, float, unsigned int, float);
void load_cal(unsigned int, float*, unsigned int, float*, unsigned int, float*);
void save_cal_all();
void load_cal_all();

void setup()
{
#if LOGGING > 0
	Serial.begin(9600); // for testing
#endif

	// lcd initialization
	lcd.init();
	lcd.backlight();
	lcd.print("please wait");

	cli(); // stops interrupts

	// sets PA to PA (pin ) as output
	DDRA |= B11100000;

	// sets timer 3 - for puller
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30); // fast PWM, OCR4A TOP, prescaler 8
	TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
	TIMSK3 |= B00000010; // enable compare match 3A

	// sets timer 4
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4B |= B00000010; // set prescaler to 8
	TIMSK4 |= B00001110; // enable compare match 4A, 4B, 4C for heaters

	// sets timer 5
	TCCR5A = 0;
	TCCR5B = 0;
	TCCR5B |= B00000010; // set prescaler to 8
	TIMSK5 |= B00000010; // enable compare match 5A for timer reset

	// PWM for motor
	OCR3A = 62000;

	// firing delays rising edge
	OCR4A = 0;
	OCR4B = 0;
	OCR4C = 0;

	// firing delays falling edge
	OCR5A = pulse_reset_delay;

	sei(); // continue interrupts

	pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), reset_timer, RISING);
	encoder = new RotaryEncoder(ROTARY_1A, ROTARY_1B, RotaryEncoder::LatchMode::FOUR3);
	attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), buttonPressed, FALLING); // reads button after release
	attachInterrupt(digitalPinToInterrupt(ROTARY_1A), checkPosition, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_1B), checkPosition, CHANGE);

	// PID settings
	heaterA.Set_setpoint(0);
	heaterB.Set_setpoint(0);
	heaterC.Set_setpoint(0);

	heaterA.Set_kPID(200, 7, 1, REVERSE);			// to be overwritten by saved calibration
	heaterB.Set_kPID(200, 7, 1, REVERSE);
	heaterC.Set_kPID(200, 7, 1, REVERSE);

	heaterA.Range(pulse_delay_min, pulse_delay_max);
	heaterB.Range(pulse_delay_min, pulse_delay_max);
	heaterC.Range(pulse_delay_min, pulse_delay_max);

	heaterA.PID_I_disableatError = 5;
	heaterB.PID_I_disableatError = 5;
	heaterC.PID_I_disableatError = 5;

	puller.Set_setpoint(1.75);
	puller.Set_kPID(100000, 5000, 500, DIRECT);		// to be overwritten by saved calibration
	puller.Range(motor_pulse_delay_min, motor_pulse_delay_max);
	puller.PID_I_reset = false;

	// EEPROM INDEX
	heaterA.EEPROM_idx(4, 8, 12, 16);
	heaterB.EEPROM_idx(20, 24, 28, 32);
	heaterC.EEPROM_idx(36, 40, 44, 48);
	puller.EEPROM_idx(52, 56, 60, 64);
	// uint8_t dia_analog_val_idx[6] = {68, 72, 76, 80, 84, 88};
	// uint8_t dia_size_cal_idx[6] = {92, 96, 100, 104, 108, 112};

	// hall sensor
	pinMode(HALL_SENSOR_PIN, INPUT);
	pinMode(TACHO, INPUT_PULLUP);
	pinMode(STEPPER_PULSE, OUTPUT);
	pinMode(MOTOR_PULSE, OUTPUT);


	STEPPER_RUN();
	MOTOR_RUN();

	// load EEPROM
	// save_cal_all();
	load_set();
	load_cal_all();	

	// end lcd startup	
	lcd.clear();
	cursor(2, 6);
	lcd.print("FILAMENT");
	cursor(3, 6);
	lcd.print("EXTRUDER");
	delay(2000);

	lcd.clear();

	cursor(2, 6);
	lcd.print("BSME 4-2");
	cursor(3, 5);
	lcd.print("2018  2022");
	delay(2000);

	// cursor()
	cursor(8, 1);
	lcd.print("pls po ty");
	delay(1000);

	lcd.clear();
}

void loop()
{
	// if (hazard)
	// {
	//   // STOP ALL
	// }
	currentMillis = millis();
	// PORTA |= B00010000;
	// digitalWrite(MOTOR_PULSE, HIGH);
	menuleveler();
	display_lcd();
	read_RPM();
	stepRPM = 600000 / OCR3A;
	stepRPM_string = stepRPM;

	if (currentMillis - previousMillis_read_dia >= Delay_read_dia)
	{
		previousMillis_read_dia = currentMillis;
		ANALOG_AVE(analogRead(HALL_SENSOR_PIN), &analog_ave);
	}

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
		if (currentMillis - previousMillis_logging >= Delay_logging)
		{
			int dutyA = ((pulse_delay_max - heaterA.Pulse_Delay) / pulse_reset_delay) * 100;
			int dutyB = ((pulse_delay_max - heaterB.Pulse_Delay) / pulse_reset_delay) * 100;
			int dutyC = ((pulse_delay_max - heaterC.Pulse_Delay) / pulse_reset_delay) * 100;

			previousMillis_logging = currentMillis;
			Serial.print(heaterA.Input);
			Serial.print(", ");
			Serial.print(dutyA);
			Serial.print(", ");
			Serial.print(heaterB.Input);
			Serial.print(", ");
			Serial.print(dutyB);
			Serial.print(", ");
			Serial.print(heaterC.Input);
			Serial.print(", ");
			Serial.print(dutyC);
			Serial.print(", ");
			Serial.print(RPM);
			Serial.print(", ");
			Serial.print(stepRPM);
			Serial.print(", ");
			Serial.print(convert2dia(analog_ave));
			Serial.println();
		}
	}

}

// function definitions
void reset_timer()
{
	PORTA &= !B11100000;
	zero_cross = true;
	TCNT4 = 0;
	TCNT5 = 0;
}

void heater_loop()
{
	heaterA.Input = thermoA.readCelsius(); // store temp reading to ACPID.Input
	heaterB.Input = thermoB.readCelsius();
	heaterC.Input = thermoC.readCelsius();

	if (start_stop)
	{
		heaterA.Compute(Delay_readtemp);
		heaterB.Compute(Delay_readtemp);
		heaterC.Compute(Delay_readtemp);

		if (heaterA.Input > 0 || TEST_MODE) // check if thermocouple disconnects
		{
			OCR4A = heaterA.Pulse_Delay;
		}
		else
		{
			OCR4A = 65535; // turn heater off if thermocouple is off
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

#if LOGGING >= 5 // DEBUG 5
	// Serial.println(OCR4A);
	// Serial.println(OCR4B);
	// Serial.println(OCR4C);
#endif
}

void puller_loop()
{
	puller.Input = convert2dia(analog_ave);
	if (stepper_run)
	{
		puller.Compute(Delay_puller);
		OCR3A = puller.Pulse_Delay;
	}
}

float convert2dia(float in)
{
	// converts an ADC reading to diameter
	// Inspired by Sprinter / Marlin thermistor reading
	byte numtemps = 6;
	float table[numtemps][2] = {
		//{ADC reading in, diameter out}

		// REPLACE THESE WITH YOUR OWN READINGS AND DRILL BIT DIAMETERS

		{dia_analog_val[0], dia_size_cal[0]},		// safety
		{dia_analog_val[1], dia_size_cal[1]}, 		// 2mm drill bit
		{dia_analog_val[2], dia_size_cal[2]},	 	// 1.75 mm
		{dia_analog_val[3], dia_size_cal[3]},		// 1.6 mm
		{dia_analog_val[4], dia_size_cal[4]}, 		// 1.2 mm
		{dia_analog_val[5], dia_size_cal[5]} 		// safety
	};
	byte i;
	float out;
	for (i = 1; i < numtemps; i++)
	{
		// check if we've found the appropriate row
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

void ANALOG_AVE(int reading, int *output)
{
	static int vol_arr[31];
	int vol_sum = 0;
	vol_arr[30] = reading;
	for (uint8_t i = 0; i < 30; i++)
	{
		vol_sum += vol_arr[i];
		vol_arr[i] = vol_arr[i + 1];
	}
	*output = int(vol_sum / 30);
}

float read_RPM()
{
	static bool last_tacho = true;
	static unsigned long previousTacho;
	static unsigned long currentTacho;
	static float Tachotime;
	currentTacho = millis();
	if (digitalRead(TACHO) != last_tacho)
	{
		Tachotime = currentTacho - previousTacho;
		previousTacho = currentTacho;
		last_tacho = !last_tacho;
	}
	if (currentTacho - previousTacho > tacho_timeout)
	{
		Tachotime = 0;
		// motor_run = false;
	}		
	float ret = 60000 / (4 * Tachotime);
	RPM = ret;
	RPM_string = ret;
	return ret; // 1000 ms/s * 60 s  / ms
}

void STEPPER_RUN()
{
	if (stepper_run)
	{

		// noInterrupts();
		// OCR3A = 16000;
		TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30); // fast PWM, OCR4A TOP, prescaler 8
		TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
		TIMSK3 |= B00000010; // enable compare match 3A
		// interrupts();
		// PORTA |= B00010000;
		// puller.PID_I = 65535;
		// digitalWrite(MOTOR_PULSE, HIGH);		// AC motor run
	}
	else
	{
		// noInterrupts();
		TCCR3A = 0;
		TCCR3B = 0;
		// TCCR3A &= !B01000000;
		TIMSK3 &= !B00000010; // disable compare match 4A, 4B, 4C for heaters
		// OCR3A = 16000;
		// // puller.PID_I = 65535;
		// interrupts();
		// PORTA &= !B00010000;
		// digitalWrite(MOTOR_PULSE, LOW);		// AC motor run
	}
}

void MOTOR_RUN()
{
	if (motor_run)
	{
		digitalWrite(MOTOR_PULSE, HIGH);		// AC motor run
	}
	else
	{
		digitalWrite(MOTOR_PULSE, LOW);			// AC motor run
	}
	
	
}

void START_STOP()
{
	if (start_stop)
	{
		TIMSK4 |= B00001110; // enable compare match 4A, 4B, 4C for heaters
	}
	else
	{
		TIMSK4 &= !B00001110; // disable compare match 4A, 4B, 4C for heaters
		heaterA.PID_I = pulse_delay_max;
		heaterA.PID_I = pulse_delay_max;
		heaterA.PID_I = pulse_delay_max;
	}
}

void save_set()
{
	EEPROM.put(heaterA.idx_Set, heaterA.Setpoint);
	EEPROM.put(heaterB.idx_Set, heaterB.Setpoint);
	EEPROM.put(heaterC.idx_Set, heaterC.Setpoint);
	EEPROM.put(puller.idx_Set, puller.Setpoint);
}

void load_set()
{
	EEPROM.get(heaterA.idx_Set, heaterA.Setpoint);
	EEPROM.get(heaterB.idx_Set, heaterB.Setpoint);
	EEPROM.get(heaterC.idx_Set, heaterC.Setpoint);
	EEPROM.get(puller.idx_Set, puller.Setpoint);
}

void save_cal(unsigned int idx_kP, float kP, unsigned int idx_kI, float kI, unsigned int idx_kD, float kD)
{
	EEPROM.put(idx_kP, kP);
	EEPROM.put(idx_kI, kI);
	EEPROM.put(idx_kD, kD);
}

void load_cal(unsigned int idx_kP, float* kP, unsigned int idx_kI, float* kI, unsigned int idx_kD, float* kD)
{
	EEPROM.get(idx_kP, *kP);
	EEPROM.get(idx_kI, *kI);
	EEPROM.get(idx_kD, *kD);
}

void save_cal_sizesensor()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		EEPROM.put(dia_analog_val_idx[i], dia_analog_val[i]);
		EEPROM.put(dia_size_cal_idx[i], dia_size_cal[i]);
	}
}

void load_cal_sizesensor()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		EEPROM.get(dia_analog_val_idx[i], dia_analog_val[i]);
		EEPROM.get(dia_size_cal_idx[i], dia_size_cal[i]);
	}
}

void save_cal_all()
{
	save_cal(heaterA.idx_kP, heaterA.kP, heaterA.idx_kI, heaterA.kI, heaterA.idx_kD, heaterA.kD);
	save_cal(heaterB.idx_kP, heaterB.kP, heaterB.idx_kI, heaterB.kI, heaterB.idx_kD, heaterB.kD);
	save_cal(heaterC.idx_kP, heaterC.kP, heaterC.idx_kI, heaterC.kI, heaterC.idx_kD, heaterC.kD);
	save_cal(puller.idx_kP, puller.kP, puller.idx_kI, puller.kI, puller.idx_kD, puller.kD);		
	save_cal_sizesensor();
}

void load_cal_all()
{
	load_cal(heaterA.idx_kP, &heaterA.kP, heaterA.idx_kI, &heaterA.kI, heaterA.idx_kD, &heaterA.kD);
	load_cal(heaterB.idx_kP, &heaterB.kP, heaterB.idx_kI, &heaterB.kI, heaterB.idx_kD, &heaterB.kD);
	load_cal(heaterC.idx_kP, &heaterC.kP, heaterC.idx_kI, &heaterC.kI, heaterC.idx_kD, &heaterC.kD);
	load_cal(puller.idx_kP, &puller.kP, puller.idx_kI, &puller.kI, puller.idx_kD, &puller.kD);
	load_cal_sizesensor();
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

void check_mark(bool check, uint8_t printlevel)
{
	if (check)
	{
		cursor(printlevel, 17);
		lcd.print("[/]");
	}
	else
	{
		cursor(printlevel, 17);
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
		}
		else
		{
			display_valuesetter = true;
		}

		// check the menu level
		if (menulevel[0] != 0)
		{
			if (menulevel[1] != 0)
			{
				if (menulevel[2] != 0)
				{
					if (menulevel[3] != 0)
					{
						if (menulevel[4] != 0)
						{
							/* code */
						}
						else
						{
							menulevel[4] = oldposition;
						}
						
						
					}
					else
					{
						menulevel[3] = oldposition;
					}
				}
				else
				{
					menulevel[2] = oldposition;
				}
			}
			else
			{
				menulevel[1] = oldposition;
			}
		}
		else
		{
			menulevel[0] = oldposition;
		}

		// reset button state
		isButton = false;

#if LOGGING >= 6
		for (uint8_t i = 0; i < 5; i++)
		{
			Serial.print(menulevel[i]);
			Serial.print(" ");
		}
		Serial.println();
#endif
	}
}


void display_Setter(float *value, float multiplier, uint8_t printlevel, String label)
{
	if (display_dynamic)
	{
		cursor(printlevel, label.length() + 2);
		lcd.print(*value);
		lcd.print(" ");
	}

	if (display_static)
	{
		cursor(printlevel, 1);
		lcd.print(label);
		encoder->getDirection(); // resets value of direction before using it to set
	}
	*value += (int(encoder->getDirection()) * multiplier);
	display_static = false;
}

void display_Setter_sizesensor(float* size_cal, float* analog_val, float multiplier, uint8_t printlevel, String label)
{	
	if (display_dynamic)
	{
		cursor(printlevel, 1);
		lcd.print(*size_cal);
		lcd.print(" ");
		cursor(printlevel, 6);
		lcd.print(analog_ave);
		lcd.print(" ");
	}

	if (display_static)
	{
		cursor(1, 0);
		lcd.print(label);
		encoder->getDirection(); // resets value of direction before using it to set
	}
	*size_cal += (int(encoder->getDirection()) * multiplier);
	*analog_val = analog_ave;
	display_static = false;
}

void display_lcd()
{
	switch (menulevel[0])
	{
	case 0: // Main Menu
		selector(4);
		// run only once to save processing time
		if (display_static)
		{

			cursor(1, 1);
			lcd.print("Extrude");
			cursor(2, 1);
			lcd.print("Calibrate");
			cursor(3, 1);
			lcd.print("Settings");
			cursor(4, 1);
			lcd.print("About");
			display_static = false;
		}
		break;

	case 1: // extrude
		switch (menulevel[1])
		{
		case 0: // extrude/
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
			break;

		case 1: // extrude/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			// display_MainMenu();
			break;

		case 2: // extrude/set temp
			switch (menulevel[2])
			{
			case 0: // extrude/set temp/
				selector(8);
				// display_Menu_2_2();
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
					cursor(6, 1);
					lcd.print("Size");
					cursor(7, 1);
					lcd.print("SAVE");
					cursor(8, 1);
					lcd.print("RESET");
					// set temps
					cursor(2, 4);
					lcd.print(heaterA.Setpoint);
					cursor(3, 4);
					lcd.print(heaterB.Setpoint);
					cursor(4, 4);
					lcd.print(heaterC.Setpoint);
					cursor(6, 6);
					lcd.print(puller.Setpoint);
					display_static = false;
				}
				break;

			case 1: // extrude/set temp/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				// display_Menu_2();
				break;

			case 2: // extrude/set temp/T1
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				// display_Set_heaterA();
				display_Setter(&heaterA.Setpoint, 1, 2, "T1");
				break;

			case 3: // extrude/set temp/T2
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				// display_Set_heaterB();
				display_Setter(&heaterB.Setpoint, 1, 3, "T2");
				break;

			case 4: // extrude/set temp/T3
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&heaterC.Setpoint, 1, 4, "T3");
				break;

			case 6: // extrude/set temp/Size
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(5);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&puller.Setpoint, 0.01, 6, "Size");
				break;

			case 7:
				save_set();
				encoder->setPosition(1);
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;
			
			case 8:
				load_set();
				menulevel[2] = 0;
				break;

			default:
				menulevel[2] = 0;
				break;
			}
			break;

		case 3: // extrude/start
			switch (menulevel[2])
			{
			case 0: // extrude/start/
				// display_Menu_2_3();
				selector(8);
				// add heater read temps
				if (display_dynamic)
				{
					cursor(2, 4);
					if (heaterA.Input < 800)
						lcd.print(heaterA.Input);
					lcd.print(" ");
					cursor(3, 4);
					if (heaterB.Input < 800)
						lcd.print(heaterB.Input);
					lcd.print(" ");
					cursor(4, 4);
					if (heaterC.Input < 800)
						lcd.print(heaterC.Input);
					lcd.print(" ");
					cursor(5, 6);
					if (motor_run)
						lcd.print(RPM_string.substring(0,4));
					cursor(6, 6);
					if (stepper_run)
						lcd.print(stepRPM_string.substring(0,4));
					cursor(7, 6);
					// lcd.print(analog_ave);
					lcd.print(convert2dia(analog_ave));
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
						lcd.print("Motr ON");
					}
					else
					{
						lcd.print("Motor OFF");
					}
					cursor(6, 1);
					if (stepper_run)
					{
						lcd.print("Stpr ON");
					}
					else
					{
						lcd.print("Stepr OFF");
					}
					// lcd.print("RPM");
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
				break;

			case 1: // extrude/start/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				// display_Menu_2();
				break;

			case 2: // extrude/start/T1
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				// display_Set_heaterA();
				display_Setter(&heaterA.Setpoint, 1, 2, "T1");
				break;

			case 3: // extrude/start/T2
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				// display_Set_heaterB();
				display_Setter(&heaterB.Setpoint, 1, 3, "T2");
				break;

			case 4: // extrude/start/T3
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&heaterC.Setpoint, 1, 4, "T3");
				break;

			case 5: // extrude/start/run motor
				motor_run = !motor_run;
				MOTOR_RUN();
				// menulevel[1] = 0;
				display_static = true;
				menulevel[2] = 0;
				break;

			case 6: // extrude/start/run stepper
				stepper_run = !stepper_run;
				STEPPER_RUN();
				// menulevel[1] = 0;
				display_static = true;
				menulevel[2] = 0;
				break;

			case 8: // extrude/start/start htr
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

	case 2: // calibrate
		switch (menulevel[1])
		{
		case 0:
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
			break;

		case 1: // calibrate/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		case 2:	// calibrate/heater PID
			switch (menulevel[2])
			{
			case 0:
				selector(4);
				// run only once to save processing time
				if (display_static)
				{
					cursor(1, 1);
					lcd.print("back");
					cursor(2, 1);
					lcd.print("heater 1");
					cursor(3, 1);
					lcd.print("heater 2");
					cursor(4, 1);
					lcd.print("heater 3");
					display_static = false;
				}
				break;
			
			case 1: // calibrate/heater PID/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;

			case 2:	// calibrate/heater PID/heater A
				switch (menulevel[3])
				{
				case 0:
					selector(6);
					// run only once to save processing time
					if (display_static)
					{
						cursor(1, 1);
						lcd.print("back");
						cursor(2, 1);
						lcd.print("kP");
						cursor(3, 1);
						lcd.print("kI");
						cursor(4, 1);
						lcd.print("kD");
						cursor(5, 1);
						lcd.print("SAVE");
						cursor(6, 1);
						lcd.print("RESET");
						cursor(2, 4);
						lcd.print(round(heaterA.kP));
						cursor(3, 4);
						lcd.print(round(heaterA.kI));
						cursor(4, 4);
						lcd.print(round(heaterA.kD));
						display_static = false;
					}
					break;
				
				case 1: // calibrate/heater PID/heater A/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterA.kP, 1, 2, "kP");
					break;	

				case 3:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterA.kI, 1, 3, "kI");
					break;	

				case 4:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterA.kD, 1, 4, "kD");
					break;	

				case 5:
					save_cal(heaterA.idx_kP, heaterA.kP, heaterA.idx_kI, heaterA.kI, heaterA.idx_kD, heaterA.kD);
					encoder->setPosition(1);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;
				
				case 6:
					load_cal(heaterA.idx_kP, &heaterA.kP, heaterA.idx_kI, &heaterA.kI, heaterA.idx_kD, &heaterA.kD);
					encoder->setPosition(1);
					// menulevel[2] = 0;
					menulevel[3] = 0;
					break;			

				default:
					menulevel[3] = 0;
					break;	
				}
				break;
			
			case 3: // calibrate/heater PID/Heater B
				switch (menulevel[3])
				{
				case 0:
					selector(6);
					// run only once to save processing time
					if (display_static)
					{
						cursor(1, 1);
						lcd.print("back");
						cursor(2, 1);
						lcd.print("kP");
						cursor(3, 1);
						lcd.print("kI");
						cursor(4, 1);
						lcd.print("kD");
						cursor(5, 1);
						lcd.print("SAVE");
						cursor(6, 1);
						lcd.print("RESET");
						cursor(2, 4);
						lcd.print(round(heaterB.kP));
						cursor(3, 4);
						lcd.print(round(heaterB.kI));
						cursor(4, 4);
						lcd.print(round(heaterB.kD));
						display_static = false;
					}
					break;
				
				case 1: // calibrate/heater PID/heater A/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterB.kP, 1, 2, "kP");
					break;	

				case 3:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterB.kI, 1, 3, "kI");
					break;	

				case 4:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterB.kD, 1, 4, "kD");
					break;	

				case 5:
					save_cal(heaterB.idx_kP, heaterB.kP, heaterB.idx_kI, heaterB.kI, heaterB.idx_kD, heaterB.kD);
					encoder->setPosition(2);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;

				case 6:
					load_cal(heaterB.idx_kP, &heaterB.kP, heaterB.idx_kI, &heaterB.kI, heaterB.idx_kD, &heaterB.kD);
					encoder->setPosition(2);
					// menulevel[2] = 0;
					menulevel[3] = 0;
					break;
				
				default:
					menulevel[3] = 0;
					break;	
				}
				break;

			case 4:	// calibrate/heater PID/Heater C
				switch (menulevel[3])
				{
				case 0:
					selector(6);
					// run only once to save processing time
					if (display_static)
					{
						cursor(1, 1);
						lcd.print("back");
						cursor(2, 1);
						lcd.print("kP");
						cursor(3, 1);
						lcd.print("kI");
						cursor(4, 1);
						lcd.print("kD");
						cursor(5, 1);
						lcd.print("SAVE");
						cursor(6, 1);
						lcd.print("RESET");
						cursor(2, 4);
						lcd.print(round(heaterC.kP));
						cursor(3, 4);
						lcd.print(round(heaterC.kI));
						cursor(4, 4);
						lcd.print(round(heaterC.kD));
						display_static = false;
					}
					break;
				
				case 1: // calibrate/heater PID/heater A/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterC.kP, 1, 2, "kP");
					break;	

				case 3:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterC.kI, 1, 3, "kI");
					break;	

				case 4:	
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					// display_Set_heaterC();
					display_Setter(&heaterC.kD, 1, 4, "kD");
					break;	

				case 5:
					save_cal(heaterC.idx_kP, heaterC.kP, heaterC.idx_kI, heaterC.kI, heaterC.idx_kD, heaterC.kD);
					encoder->setPosition(3);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;
				
				case 6:
					load_cal(heaterC.idx_kP, &heaterC.kP, heaterC.idx_kI, &heaterC.kI, heaterC.idx_kD, &heaterC.kD);
					encoder->setPosition(3);
					// menulevel[2] = 0;
					menulevel[3] = 0;
					break;

				default:
					menulevel[3] = 0;
					break;	
				}
				break;

			default:
				menulevel[2] = 0;
				break;
			}
			break;

		case 3:
			switch (menulevel[2])
			{
			case 0:
				selector(6);
				// run only once to save processing time
				if (display_static)
				{
					cursor(1, 1);
					lcd.print("back");
					cursor(2, 1);
					lcd.print("kP");
					cursor(3, 1);
					lcd.print("kI");
					cursor(4, 1);
					lcd.print("kD");
					cursor(5, 1);
					lcd.print("SAVE");
					cursor(6, 1);
					lcd.print("RESET");
					cursor(2, 4);
					lcd.print(round(puller.kP));
					cursor(3, 4);
					lcd.print(round(puller.kI));
					cursor(4, 4);
					lcd.print(round(puller.kD));
					display_static = false;
				}
				break;
			
			case 1: // calibrate/heater PID/heater A/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;	

			case 2:	
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&puller.kP, 100, 2, "kP");
				break;	

			case 3:	
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&puller.kI, 10, 3, "kI");
				break;	

			case 4:	
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				// display_Set_heaterC();
				display_Setter(&puller.kD, 10, 4, "kD");
				break;	

			case 5:
				save_cal(puller.idx_kP, puller.kP, puller.idx_kI, puller.kI, puller.idx_kD, puller.kD);
				encoder->setPosition(2);
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;
			
			case 6:
				load_cal(puller.idx_kP, &puller.kP, puller.idx_kI, &puller.kI, puller.idx_kD, &puller.kD);
				encoder->setPosition(2);
				// menulevel[1] = 0;
				menulevel[2] = 0;
				break;

			default:
				menulevel[2] = 0;
				break;	
			}
			break;

		case 4:
			switch (menulevel[2])
			{
			case 0:
				selector(8);
				if (display_static)
				{
					cursor(1, 1);
					lcd.print("back");
					cursor(7, 1);
					lcd.print("SAVE");
					cursor(8, 1);
					lcd.print("RESET");
					cursor(2, 1);
					lcd.print(dia_size_cal[1]);
					cursor(2, 6);
					lcd.print(int(dia_analog_val[1]));
					cursor(3, 1);
					lcd.print(dia_size_cal[2]);
					cursor(3, 6);
					lcd.print(int(dia_analog_val[2]));
					cursor(4, 1);
					lcd.print(dia_size_cal[3]);
					cursor(4, 6);
					lcd.print(int(dia_analog_val[3]));
					cursor(6, 1);
					lcd.print(dia_size_cal[4]);
					cursor(6, 6);
					lcd.print(int(dia_analog_val[4]));
					display_static = false;
				}
				break;

			case 1: // calibrate/size sensor/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;	
			
			case 2:
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				display_Setter_sizesensor(&dia_size_cal[1], &dia_analog_val[1], 0.01, 2, "Calibration pin 1");
				break;	

			case 3:
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				display_Setter_sizesensor(&dia_size_cal[2], &dia_analog_val[2], 0.01, 3, "Calibration pin 2");
				break;	

			case 4:
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				display_Setter_sizesensor(&dia_size_cal[3], &dia_analog_val[3], 0.01, 4, "Calibration pin 3");
				break;	

			case 6:
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(5);
					break;
				}
				display_Setter_sizesensor(&dia_size_cal[4], &dia_analog_val[4], 0.01, 6, "Calibration pin 4");
				break;	

			case 7:
				save_cal_sizesensor();
				menulevel[2] = 0;
				break;

			case 8:
				load_cal_sizesensor();
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

	case 3: // settings
		switch (menulevel[1])
		{
		case 0: // settings/
			selector(4);
			// run only once to save processing time
			if (display_static)
			{
				cursor(1, 1);
				lcd.print("back");
				cursor(2, 1);
				lcd.print("Test mode");
				cursor(3, 1);
				// lcd.print("RPM Control");
				lcd.print("Serial logging");
				cursor(4, 1);
				lcd.print("Restore defaults");

				check_mark(TEST_MODE, 2);
				// check_mark(control_RPM, 3);
				check_mark(SERIAL_LOGGING, 3);

				display_static = false;
			}
			break;

		case 1: // settings/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			// display_MainMenu();
			break;

		case 2: // settings/Test mode
			TEST_MODE = !TEST_MODE;
			display_static = true;
			menulevel[1] = 0;
			break;

		case 3: // settings/serial logging
			SERIAL_LOGGING = !SERIAL_LOGGING;
			display_static = true;
			menulevel[1] = 0;
			break;

		case 4: // settings/restore defaults
			encoder->setPosition(2);
			heaterA.Set_setpoint(0);
			heaterB.Set_setpoint(0);
			heaterC.Set_setpoint(0);
			puller.Set_setpoint(1.75);
			save_set();
			heaterA.Set_kPID(200, 7, 1, REVERSE);
			heaterB.Set_kPID(200, 7, 1, REVERSE);
			heaterC.Set_kPID(200, 7, 1, REVERSE);
			puller.Set_kPID(100000, 5000, 500, DIRECT);
			save_cal_all();
			float dia_analog_val_def[6] = {0, 600, 622, 692, 883, 1023};
			float dia_size_cal_def[6] = {3, 2.00, 1.75, 1.60, 1.20, 0};
			for (uint8_t i = 0; i < 6; i++)
			{
				EEPROM.put(dia_analog_val_idx[i], dia_analog_val_def[i]);
				EEPROM.put(dia_size_cal_idx[i], dia_size_cal_def[i]);
			}
			load_cal_all();
			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		default:
			menulevel[1] = 0;
			break;
		}
		break;

	case 4:
		switch (menulevel[1])
		{
		case 0:
			if (display_static)
			{
				// cursor(1, 0);
				// lcd.print("BSME 4-2");
				cursor(1, 1);
				lcd.print("BSME 4-2 batch 2022");
				cursor(2, 1);
				lcd.print("Denisse");
				cursor(3, 1);
				lcd.print("bitsuki");
				cursor(4, 2);
				lcd.print("D'AEms");
				cursor(6, 0);
				lcd.print("densanity");
				cursor(7, 1);
				lcd.print("RealED");
				cursor(8, 1);
				lcd.print("alonyx");
				display_static = false;
			}			
			break;

		default:
			menulevel[0] = 0;
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
		PORTA |= B10000000; // turns pin 29 on
	}
}

// turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect)
{
	if (zero_cross)
	{
		PORTA |= B01000000; // turns pin 28 on
	}
}

// turns on firing pulse for heater 3
ISR(TIMER4_COMPC_vect)
{
	if (zero_cross)
	{
		PORTA |= B00100000; // turns pin 27 on
	}
}

// turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect)
{
	PORTA &= !B11100000; // turns pin 29, 28, 27 off
	zero_cross = false;

	if (TEST_MODE)
	{
		reset_timer();
	}
}