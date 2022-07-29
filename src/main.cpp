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

// PIN DECLARATIONS
#define HALL_SENSOR_PIN A8						// for filament diameter sensor
#define TACHO A9								// for AC motor tachometer
const uint8_t STEPPER_PULSE = 5;				// stepper motor pulse pin (OCR3A)
const uint8_t STEPPER_ENABLE = 43;				// stepper motor enable/disable pin
const uint8_t MOTOR_PULSE = 31;					// AC motor pulse pin/or enable pin	
const uint8_t ZERO_CROSS_PIN = 18; 				// zero cross pin for hardware interrupt
const uint8_t SPI_CLOCK = 52;	   				// CLK for thermocouples
const uint8_t SPI_MISO = 50;					// SO for thermocouples
const uint8_t SPI_thermoA = 30;					// CS for thermocouple A			
const uint8_t SPI_thermoB = 32;					// CS for thermocouple B
const uint8_t SPI_thermoC = 34;					// CS for thermocouple C
const uint8_t ROTARY_BUTTON = 19;				// button for controls
const uint8_t ROTARY_1A = 2;					// for rotary encoder
const uint8_t ROTARY_1B = 3;					// for rotary encoder
// AC HEATER PULSE PINS = 29, 28, 27			// set using DDRA for faster response

// CHANGEABLE CONSTANTS
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
const int reset_heater_PID_I_at_error = 5;		// reset heater PID_I at error

// FOR TIME KEEPING, DO NOT EDIT
unsigned long currentMillis = 0;
unsigned long previousMillis_display = 0;
unsigned long previousMillis_temp = 0;
unsigned long previousMillis_puller = 0;
unsigned long previousMillis_logging = 0;
unsigned long previousMillis_read_dia = 0;

// OTHER VARIABLES FOR THE CODE, DO NOT EDIT	// DO NOT EDIT, value changes inside the code
unsigned long motor_pulsecount = 0;				// unused, uncomment on ISR comparematch 3A to count steps
int analog_ave = 0;								// average of HALL_SENSOR_PIN analog reading for filament diameter sensor
bool zero_cross = false;						// TRUE if AC passes zero cross, FALSE if heater is turned on
bool isButton = false;							// updates to TRUE if button is pressed, FALSE after action is done
bool display_static = true;						// allows single processing time for unchanging displays on LCD, TRUE if screen refreshes (or button is pressed), FALSE after displaying static displays
bool display_dynamic = true;					// used for display of values that changes over time, TRUE every set Delay_display, FALSE after displaying values
bool display_valuesetter = false;				// used when the display is in function that changes value of a variable, TRUE when it does
bool motor_run = false;							// TRUE when AC motor is enabled
bool stepper_run = false;						// TRUE when stepper motor is enabled
bool heater_run = false;						// TRUE when heater is enabled
bool TEST_MODE = false;							// TRUE when testing without AC power to check pins
bool SERIAL_LOGGING = false;					// TRUE when serial data logging is on
static int oldposition;							// saves old position of the rotary encoder
uint8_t menulevel[5] = {0, 0, 0, 0, 0};			// used for finding what to display
float RPM;										// saves AC motor RPM as float
String RPM_string;								// saves AC motor RPM as string for display
float stepRPM;									// saves stepper motor RPM as float
String stepRPM_string;							// saves stepper motor RPM as string for display
float dia_analog_val[6] = {};					// declaration of array of analog values for diameter approximation
float dia_size_cal[6] = {};						// declaration of array of values to interpolate from analog values for diameter approximation
uint8_t dia_analog_val_idx[6] = {68, 72, 76, 80, 84, 88};
												// EEPROM index for saving values of the dia_analog_val[6]
uint8_t dia_size_cal_idx[6] = {92, 96, 100, 104, 108, 112};
												// EEPROM index for saving values of the dia_size_val[6]

// CLASS INITIALIZATIONS
// FOR PIDs OF HEATERS
ACPID heaterA(REVERSE);
ACPID heaterB(REVERSE);
ACPID heaterC(REVERSE);

// FOR PID OF STEPPER MOTOR - DIAMETER
ACPID puller(DIRECT);

// FOR THERMOCOUPLES
MAX6675 thermoA(SPI_CLOCK, SPI_thermoA, SPI_MISO);
MAX6675 thermoB(SPI_CLOCK, SPI_thermoB, SPI_MISO);
MAX6675 thermoC(SPI_CLOCK, SPI_thermoC, SPI_MISO);

// FOR DISPLAY AND CONTROLS
RotaryEncoder *encoder = nullptr;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// FUNCTION DECLARATIONS
void STEPPER_RUN();								// sets stepper motor on/off
void MOTOR_RUN();								// sets AC motor on/off
void HEATER_RUN();								// sets heater on/off
void RESET_TIMER();								// resets timer interrupt timers (TIMER 4&5) used for phase control
void HEATER_LOOP();								// heater temperature reading and firing angle value updates
void PULLER_LOOP();								// updates puller PWM changing its speed
void ANALOG_AVE(int, int *);					// averages analog reading for diameter approx
float CONVERT2DIA(float);						// converts analog reading to diameter
float READ_RPM();								// reads AC motor RPM
void CHECK_POSITION();							// ISR for rotary encoder that ticks every movement of rotary 
void BUTTON_PRESSED();							// ISR for button presses
void CHECK_MARK(bool, uint8_t);					// displays check mark
void DISPLAY_CURSOR(uint8_t, uint8_t);			// set LCD DISPLAY_CURSOR position for printing displays
int8_t DISPLAY_SELECTOR(int8_t);				// prints '>' to LCD display
void DISPLAY_MENULEVELER();						// sets what to display, updates display when button is pressed
void DISPLAY_SETTER(float *, float, uint8_t, String);
												// display for value changing functions
void DISPLAY_SETTER_SIZESENSOR(float*, float*, float, uint8_t, String);
												// display for value changing functions specifically for sizesensor
void DISPLAY_LCD();								// contains all the display to print in the LCD, change here if you want to print something in the screen
void SAVE_SETPOINT();							// save setpoint values to the EEPROM
void LOAD_SETPOINT();							// load setpoint values from the EEPROM
void SAVE_CAL(unsigned int, float, unsigned int, float, unsigned int, float);
												// save PID constants for heaters and/or pullers to the EEPROM
void LOAD_CAL(unsigned int, float*, unsigned int, float*, unsigned int, float*);
												// load PID constants for heaters and/or pullers from the EEPROM
void SAVE_CAL_SIZESENSOR();						// save filament diameter sensor calibration to the EEPROM
void LOAD_CAL_SIZESENSOR();						// load filament diameter sensor calibration from the EEPROM
void SAVE_CAL_ALL();							// save all calibration to the EEPROM
void LOAD_CAL_ALL();							// save all calibration from the EEPROM


void setup()
{
#if LOGGING > 0
	Serial.begin(9600);
#endif

	lcd.init();									// LCD INITIALIZATION
	lcd.backlight();
	lcd.print("please wait");
	DISPLAY_CURSOR(2, 0);
	lcd.print("timers");

	cli(); 										// stops interrupts

	DDRA |= B11100000;							// sets PORTA PA7,PA6,PA5 as output (pins 29, 28, 27)

	// sets timer 3 - for puller
	TCCR3A = 0;
	TCCR3B = 0;
	// fast PWM, OCR4A TOP, prescaler 8, toggle OCR3A on compare match
	TCCR3A |= B01000011;						// TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30);
	TCCR3B |= B00011010;						// TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
	TIMSK3 |= B00000010; 						// enable compare match 3A
	OCR3A = 62000;								// PWM for motor test at startup
	
	// sets timer 4
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4B |= B00000010; 						// set prescaler to 8
	TIMSK4 |= B00001110; 						// enable compare match 4A, 4B, 4C for heaters

	// sets timer 5
	TCCR5A = 0;
	TCCR5B = 0;
	TCCR5B |= B00000010; 						// set prescaler to 8
	TIMSK5 |= B00000010; 						// enable compare match 5A for timer reset

	// firing pulse rising edge
	OCR4A = 0;									
	OCR4B = 0;
	OCR4C = 0;

	// firing delays falling edge
	OCR5A = pulse_reset_delay;

	sei(); 										// continue interrupts

	// SETUPS
	DISPLAY_CURSOR(3, 0);
	lcd.print("setups");

	pinMode(STEPPER_ENABLE, OUTPUT);
	pinMode(HALL_SENSOR_PIN, INPUT);
	pinMode(TACHO, INPUT_PULLUP);
	pinMode(STEPPER_PULSE, OUTPUT);
	pinMode(MOTOR_PULSE, OUTPUT);

	// zero cross detection interrupt setup
	pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), RESET_TIMER, RISING);

	// rotary encoder interrupt setups; // reads button after release
	encoder = new RotaryEncoder(ROTARY_1A, ROTARY_1B, RotaryEncoder::LatchMode::FOUR3);
	attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), BUTTON_PRESSED, FALLING); 
	attachInterrupt(digitalPinToInterrupt(ROTARY_1A), CHECK_POSITION, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_1B), CHECK_POSITION, CHANGE);

	// PID settings
	heaterA.Range(pulse_delay_min, pulse_delay_max);
	heaterB.Range(pulse_delay_min, pulse_delay_max);
	heaterC.Range(pulse_delay_min, pulse_delay_max);

	heaterA.PID_I_resetatError = reset_heater_PID_I_at_error;
	heaterB.PID_I_resetatError = reset_heater_PID_I_at_error;
	heaterC.PID_I_resetatError = reset_heater_PID_I_at_error;

	puller.Range(motor_pulse_delay_min, motor_pulse_delay_max);
	puller.PID_I_reset = false;

	// LOAD SETTINGS FROM THE EEPROM
	DISPLAY_CURSOR(4, 0);
	lcd.print("load EEPROM");

	// EEPROM INDEX
	heaterA.EEPROM_idx(4, 8, 12, 16);
	heaterB.EEPROM_idx(20, 24, 28, 32);
	heaterC.EEPROM_idx(36, 40, 44, 48);
	puller.EEPROM_idx(52, 56, 60, 64);
	// declared outside the void setup
	// dia_analog_val_idx[6] = {68, 72, 76, 80, 84, 88};
	// dia_size_cal_idx[6] = {92, 96, 100, 104, 108, 112};

	// load EEPROM
	LOAD_SETPOINT();
	LOAD_CAL_ALL();	

	// start motors temporarily to check if they are working
	STEPPER_RUN();
	MOTOR_RUN();

	// end lcd startup
	lcd.clear();
	DISPLAY_CURSOR(2, 6);
	lcd.print("FILAMENT");
	DISPLAY_CURSOR(3, 6);
	lcd.print("EXTRUDER");
	delay(2000);

	lcd.clear();

	DISPLAY_CURSOR(2, 6);
	lcd.print("BSME 4-2");
	DISPLAY_CURSOR(3, 5);
	lcd.print("BATCH 2022");
	delay(2000);

	lcd.clear();
}

void loop()
{
	currentMillis = millis();					// for time keeping

	DISPLAY_MENULEVELER();						// for display
	DISPLAY_LCD();
	
	// read RPM of AC motor and stepper motor if possible
	READ_RPM();
	stepRPM = 600000 / OCR3A;
	stepRPM_string = stepRPM;

	// read diameter every Delay_read_dia
	if (currentMillis - previousMillis_read_dia >= Delay_read_dia)
	{
		previousMillis_read_dia = currentMillis;
		ANALOG_AVE(analogRead(HALL_SENSOR_PIN), &analog_ave);
	}

	// read temperature and updates firing angle every Delay_readtemp
	if (currentMillis - previousMillis_temp >= Delay_readtemp)
	{
		previousMillis_temp = currentMillis;
		HEATER_LOOP();
	}

	// update stepper motor speed every Delay_puller
	if (currentMillis - previousMillis_puller >= Delay_puller)
	{
		previousMillis_puller = currentMillis;
		PULLER_LOOP();
	}

	// prints on LCD every Delay_display
	if (currentMillis - previousMillis_display >= Delay_display)
	{
		previousMillis_display = currentMillis;
		display_dynamic = true;
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
			Serial.print(CONVERT2DIA(analog_ave));
			Serial.println();
		}
	}

}

// FUNCTION DEFINITIONS
void HEATER_LOOP()
{
	heaterA.Input = thermoA.readCelsius(); 		// store temp reading to ACPID.Input
	heaterB.Input = thermoB.readCelsius();
	heaterC.Input = thermoC.readCelsius();

	if (heater_run)
	{
		heaterA.Compute(Delay_readtemp);
		heaterB.Compute(Delay_readtemp);
		heaterC.Compute(Delay_readtemp);

		if (heaterA.Input > 0 || TEST_MODE) 	// check if thermocouple disconnects
		{
			OCR4A = heaterA.Pulse_Delay;		// if still connected then update firing pulse angle
		}
		else
		{
			OCR4A = 65535; 						// turn heater off if thermocouple is off
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

void PULLER_LOOP()
{
	puller.Input = CONVERT2DIA(analog_ave);
	if (stepper_run)
	{
		puller.Compute(Delay_puller);
		OCR3A = puller.Pulse_Delay;
	}
}

float CONVERT2DIA(float in)
{
	// converts an ADC reading to diameter
	// Inspired by Sprinter / Marlin thermistor reading
	byte numtemps = 6;
	float table[numtemps][2] = {
		//{ADC reading in, diameter out}
		{dia_analog_val[0], dia_size_cal[0]},	// safety
		{dia_analog_val[1], dia_size_cal[1]}, 		
		{dia_analog_val[2], dia_size_cal[2]},	 	
		{dia_analog_val[3], dia_size_cal[3]},		
		{dia_analog_val[4], dia_size_cal[4]}, 		
		{dia_analog_val[5], dia_size_cal[5]} 	// safety
	};
	for (uint8_t i = 1; i < numtemps; i++)
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
	static int vol_arr[31];						// averages 31 analog readings
	int vol_sum = 0;
	vol_arr[30] = reading;
	for (uint8_t i = 0; i < 30; i++)
	{
		vol_sum += vol_arr[i];
		vol_arr[i] = vol_arr[i + 1];
	}
	*output = int(vol_sum / 30);
}

float READ_RPM()
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
		// motor_run = false if you want to have safety off
	}		
	float ret = 60000 / (4 * Tachotime);
	RPM = ret;
	RPM_string = ret;
	return ret; 		// 1000 ms/s * 60 s  / ms
}

void STEPPER_RUN()
{
	if (stepper_run)
	{
		digitalWrite(STEPPER_ENABLE, LOW);		// enable A4988 stepper motor driver
		OCR3A = 1;								// resets timer before enabling
		TCCR3A |= B01000011;					// TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30);
		TCCR3B |= B00011010;					// TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
		TIMSK3 |= B00000010; 					// enable compare match 3A
	}
	else
	{
		digitalWrite(STEPPER_ENABLE, HIGH);		// disable A4988 stepper motor driver
		TCCR3A = 0;								// disable timer3
		TCCR3B = 0;								// disable timer3
		TIMSK3 &= !B00000010; 					// disable compare match 3A
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
		digitalWrite(MOTOR_PULSE, LOW);			// AC motor stop
	}
	
	
}

void HEATER_RUN()
{
	if (heater_run)
	{
		TIMSK4 |= B00001110; 					// enable compare match 4A, 4B, 4C for heaters
	}
	else
	{
		TIMSK4 &= !B00001110; 					// disable compare match 4A, 4B, 4C for heaters
		heaterA.PID_I = pulse_delay_max;		
		heaterA.PID_I = pulse_delay_max;
		heaterA.PID_I = pulse_delay_max;
	}
}

void SAVE_SETPOINT()
{
	EEPROM.put(heaterA.idx_Set, heaterA.Setpoint);
	EEPROM.put(heaterB.idx_Set, heaterB.Setpoint);
	EEPROM.put(heaterC.idx_Set, heaterC.Setpoint);
	EEPROM.put(puller.idx_Set, puller.Setpoint);
}

void LOAD_SETPOINT()
{
	EEPROM.get(heaterA.idx_Set, heaterA.Setpoint);
	EEPROM.get(heaterB.idx_Set, heaterB.Setpoint);
	EEPROM.get(heaterC.idx_Set, heaterC.Setpoint);
	EEPROM.get(puller.idx_Set, puller.Setpoint);
}

void SAVE_CAL(unsigned int idx_kP, float kP, unsigned int idx_kI, float kI, unsigned int idx_kD, float kD)
{
	EEPROM.put(idx_kP, kP);
	EEPROM.put(idx_kI, kI);
	EEPROM.put(idx_kD, kD);
}

void LOAD_CAL(unsigned int idx_kP, float* kP, unsigned int idx_kI, float* kI, unsigned int idx_kD, float* kD)
{
	EEPROM.get(idx_kP, *kP);
	EEPROM.get(idx_kI, *kI);
	EEPROM.get(idx_kD, *kD);
}

void SAVE_CAL_SIZESENSOR()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		EEPROM.put(dia_analog_val_idx[i], dia_analog_val[i]);
		EEPROM.put(dia_size_cal_idx[i], dia_size_cal[i]);
	}
}

void LOAD_CAL_SIZESENSOR()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		EEPROM.get(dia_analog_val_idx[i], dia_analog_val[i]);
		EEPROM.get(dia_size_cal_idx[i], dia_size_cal[i]);
	}
}

void SAVE_CAL_ALL()
{
	SAVE_CAL(heaterA.idx_kP, heaterA.kP, heaterA.idx_kI, heaterA.kI, heaterA.idx_kD, heaterA.kD);
	SAVE_CAL(heaterB.idx_kP, heaterB.kP, heaterB.idx_kI, heaterB.kI, heaterB.idx_kD, heaterB.kD);
	SAVE_CAL(heaterC.idx_kP, heaterC.kP, heaterC.idx_kI, heaterC.kI, heaterC.idx_kD, heaterC.kD);
	SAVE_CAL(puller.idx_kP, puller.kP, puller.idx_kI, puller.kI, puller.idx_kD, puller.kD);		
	SAVE_CAL_SIZESENSOR();
}

void LOAD_CAL_ALL()
{
	LOAD_CAL(heaterA.idx_kP, &heaterA.kP, heaterA.idx_kI, &heaterA.kI, heaterA.idx_kD, &heaterA.kD);
	LOAD_CAL(heaterB.idx_kP, &heaterB.kP, heaterB.idx_kI, &heaterB.kI, heaterB.idx_kD, &heaterB.kD);
	LOAD_CAL(heaterC.idx_kP, &heaterC.kP, heaterC.idx_kI, &heaterC.kI, heaterC.idx_kD, &heaterC.kD);
	LOAD_CAL(puller.idx_kP, &puller.kP, puller.idx_kI, &puller.kI, puller.idx_kD, &puller.kD);
	LOAD_CAL_SIZESENSOR();
}

void CHECK_MARK(bool check, uint8_t printlevel)
{
	if (check)
	{
		DISPLAY_CURSOR(printlevel, 17);
		lcd.print("[/]");
	}
	else
	{
		DISPLAY_CURSOR(printlevel, 17);
		lcd.print("[ ]");
	}
}

void DISPLAY_CURSOR(uint8_t swt, uint8_t offset = 0)
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
int8_t DISPLAY_SELECTOR(int8_t numberofselection = 0)
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
			DISPLAY_CURSOR(oldposition);
			lcd.print(" ");

			// min cursor position
			if (position <= 1)
			{
				DISPLAY_CURSOR(1);
				encoder->setPosition(0);
				oldposition = 1;
			}
			// max cursor position
			else if (position >= numberofselection)
			{
				DISPLAY_CURSOR(numberofselection);
				encoder->setPosition(numberofselection - 1);
				oldposition = numberofselection;
			}
			// in between min max
			else
			{
				DISPLAY_CURSOR(position);
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

// LCD DISPLAY FUNCTIONS
void DISPLAY_MENULEVELER()
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


void DISPLAY_SETTER(float *value, float multiplier, uint8_t printlevel, String label)
{
	if (display_dynamic)
	{
		DISPLAY_CURSOR(printlevel, label.length() + 2);
		lcd.print(*value);
		lcd.print(" ");
	}

	if (display_static)
	{
		DISPLAY_CURSOR(printlevel, 1);
		lcd.print(label);
		encoder->getDirection(); 				// resets value of direction before using it to set
	}
	*value += (int(encoder->getDirection()) * multiplier);
	display_static = false;
}

void DISPLAY_SETTER_SIZESENSOR(float* size_cal, float* analog_val, float multiplier, uint8_t printlevel, String label)
{	
	if (display_dynamic)
	{
		DISPLAY_CURSOR(printlevel, 1);
		lcd.print(*size_cal);
		lcd.print(" ");
		DISPLAY_CURSOR(printlevel, 6);
		lcd.print(analog_ave);
		lcd.print(" ");
	}

	if (display_static)
	{
		DISPLAY_CURSOR(1, 0);
		lcd.print(label);
		encoder->getDirection(); 				// resets value of direction before using it to set
	}
	*size_cal += (int(encoder->getDirection()) * multiplier);
	*analog_val = analog_ave;
	display_static = false;
}

void DISPLAY_LCD()
{
	switch (menulevel[0])
	{
	case 0: 									// Main Menu
		DISPLAY_SELECTOR(4);
		// run only once to save processing time
		if (display_static)
		{

			DISPLAY_CURSOR(1, 1);
			lcd.print("Extrude");
			DISPLAY_CURSOR(2, 1);
			lcd.print("Calibrate");
			DISPLAY_CURSOR(3, 1);
			lcd.print("Settings");
			DISPLAY_CURSOR(4, 1);
			lcd.print("About");
			display_static = false;
		}
		break;

	case 1: 									// extrude
		switch (menulevel[1])
		{
		case 0: 								// extrude/
			DISPLAY_SELECTOR(3);
			// run only once to save processing time
			if (display_static)
			{
				DISPLAY_CURSOR(1, 1);
				lcd.print("back");
				DISPLAY_CURSOR(2, 1);
				lcd.print("Set Temps");
				DISPLAY_CURSOR(3, 1);
				lcd.print("Start");
				display_static = false;
			}
			break;

		case 1: 								// extrude/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		case 2: 								// extrude/set temp
			switch (menulevel[2])
			{
			case 0: 							// extrude/set temp/
				DISPLAY_SELECTOR(8);
				if (display_static)
				{
					DISPLAY_CURSOR(1, 1);
					lcd.print("back");
					DISPLAY_CURSOR(2, 1);
					lcd.print("T1");
					DISPLAY_CURSOR(3, 1);
					lcd.print("T2");
					DISPLAY_CURSOR(4, 1);
					lcd.print("T3");
					DISPLAY_CURSOR(6, 1);
					lcd.print("Size");
					DISPLAY_CURSOR(7, 1);
					lcd.print("SAVE");
					DISPLAY_CURSOR(8, 1);
					lcd.print("RESET");
					// set temps
					DISPLAY_CURSOR(2, 4);
					lcd.print(heaterA.Setpoint);
					DISPLAY_CURSOR(3, 4);
					lcd.print(heaterB.Setpoint);
					DISPLAY_CURSOR(4, 4);
					lcd.print(heaterC.Setpoint);
					DISPLAY_CURSOR(6, 6);
					lcd.print(puller.Setpoint);
					display_static = false;
				}
				break;

			case 1: 							// extrude/set temp/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;

			case 2: 							// extrude/set temp/T1
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				DISPLAY_SETTER(&heaterA.Setpoint, 1, 2, "T1");
				break;

			case 3: 							// extrude/set temp/T2
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				DISPLAY_SETTER(&heaterB.Setpoint, 1, 3, "T2");
				break;

			case 4: 							// extrude/set temp/T3
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				DISPLAY_SETTER(&heaterC.Setpoint, 1, 4, "T3");
				break;

			case 6: 							// extrude/set temp/Size
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(5);
					break;
				}
				DISPLAY_SETTER(&puller.Setpoint, 0.01, 6, "Size");
				break;

			case 7:								// extrude/set temp/SAVE
				SAVE_SETPOINT();
				encoder->setPosition(1);
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;
			
			case 8:								// extrude/set temp/RESET
				LOAD_SETPOINT();
				menulevel[2] = 0;
				break;

			default:
				menulevel[2] = 0;
				break;
			}
			break;

		case 3: 								// extrude/start
			switch (menulevel[2])
			{
			case 0: 							// extrude/start/
				DISPLAY_SELECTOR(8);
				// add heater read temps
				if (display_dynamic)
				{
					DISPLAY_CURSOR(2, 4);
					if (heaterA.Input < 800)
						lcd.print(heaterA.Input);
					lcd.print(" ");
					DISPLAY_CURSOR(3, 4);
					if (heaterB.Input < 800)
						lcd.print(heaterB.Input);
					lcd.print(" ");
					DISPLAY_CURSOR(4, 4);
					if (heaterC.Input < 800)
						lcd.print(heaterC.Input);
					lcd.print(" ");
					DISPLAY_CURSOR(5, 6);
					if (motor_run)
						lcd.print(RPM_string.substring(0,4));
					DISPLAY_CURSOR(6, 6);
					if (stepper_run)
						lcd.print(stepRPM_string.substring(0,4));
					DISPLAY_CURSOR(7, 6);
					lcd.print(CONVERT2DIA(analog_ave));
					display_dynamic = false;
				}

				// run only once to save processing time
				if (display_static)
				{
					DISPLAY_CURSOR(1, 1);
					lcd.print("back");
					DISPLAY_CURSOR(2, 1);
					lcd.print("T1");
					DISPLAY_CURSOR(3, 1);
					lcd.print("T2");
					DISPLAY_CURSOR(4, 1);
					lcd.print("T3");
					DISPLAY_CURSOR(5, 1);
					if (motor_run)
					{
						lcd.print("Motr ON");
					}
					else
					{
						lcd.print("Motor OFF");
					}
					DISPLAY_CURSOR(6, 1);
					if (stepper_run)
					{
						lcd.print("Stpr ON");
					}
					else
					{
						lcd.print("Stepr OFF");
					}
					DISPLAY_CURSOR(7, 1);
					lcd.print("Size");
					DISPLAY_CURSOR(8, 1);
					if (heater_run)
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

			case 1: 							// extrude/start/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;

			case 2: 							// extrude/start/T1
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				DISPLAY_SETTER(&heaterA.Setpoint, 1, 2, "T1");
				break;

			case 3: 							// extrude/start/T2
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				DISPLAY_SETTER(&heaterB.Setpoint, 1, 3, "T2");
				break;

			case 4: 							// extrude/start/T3
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				DISPLAY_SETTER(&heaterC.Setpoint, 1, 4, "T3");
				break;

			case 5: 							// extrude/start/run motor
				motor_run = !motor_run;
				MOTOR_RUN();
				display_static = true;
				menulevel[2] = 0;
				break;

			case 6: 							// extrude/start/run stepper
				stepper_run = !stepper_run;
				STEPPER_RUN();
				display_static = true;
				menulevel[2] = 0;
				break;

			case 8: 							// extrude/start/start htr
				heater_run = !heater_run;
				HEATER_RUN();
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

	case 2: 									// calibrate
		switch (menulevel[1])
		{
		case 0:
			DISPLAY_SELECTOR(4);
			// run only once to save processing time
			if (display_static)
			{
				DISPLAY_CURSOR(1, 1);
				lcd.print("back");
				DISPLAY_CURSOR(2, 1);
				lcd.print("heater PID");
				DISPLAY_CURSOR(3, 1);
				lcd.print("puller PID");
				DISPLAY_CURSOR(4, 1);
				lcd.print("size sensor");
				display_static = false;
			}
			break;

		case 1: 								// calibrate/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		case 2:									// calibrate/heater PID
			switch (menulevel[2])
			{
			case 0:
				DISPLAY_SELECTOR(4);
				// run only once to save processing time
				if (display_static)
				{
					DISPLAY_CURSOR(1, 1);
					lcd.print("back");
					DISPLAY_CURSOR(2, 1);
					lcd.print("heater 1");
					DISPLAY_CURSOR(3, 1);
					lcd.print("heater 2");
					DISPLAY_CURSOR(4, 1);
					lcd.print("heater 3");
					display_static = false;
				}
				break;
			
			case 1: 							// calibrate/heater PID/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;

			case 2:								// calibrate/heater PID/heater A
				switch (menulevel[3])
				{
				case 0:
					DISPLAY_SELECTOR(6);
					// run only once to save processing time
					if (display_static)
					{
						DISPLAY_CURSOR(1, 1);
						lcd.print("back");
						DISPLAY_CURSOR(2, 1);
						lcd.print("kP");
						DISPLAY_CURSOR(3, 1);
						lcd.print("kI");
						DISPLAY_CURSOR(4, 1);
						lcd.print("kD");
						DISPLAY_CURSOR(5, 1);
						lcd.print("SAVE");
						DISPLAY_CURSOR(6, 1);
						lcd.print("RESET");
						DISPLAY_CURSOR(2, 4);
						lcd.print(round(heaterA.kP));
						DISPLAY_CURSOR(3, 4);
						lcd.print(round(heaterA.kI));
						DISPLAY_CURSOR(4, 4);
						lcd.print(round(heaterA.kD));
						display_static = false;
					}
					break;
				
				case 1: 						// calibrate/heater PID/heater A/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:							// calibrate/heater PID/heater A/kP
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					DISPLAY_SETTER(&heaterA.kP, 1, 2, "kP");
					break;	

				case 3:							// calibrate/heater PID/heater A/kI
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					DISPLAY_SETTER(&heaterA.kI, 1, 3, "kI");
					break;	

				case 4:							// calibrate/heater PID/heater A/kD
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					DISPLAY_SETTER(&heaterA.kD, 1, 4, "kD");
					break;	

				case 5:							// calibrate/heater PID/heater A/SAVE
					SAVE_CAL(heaterA.idx_kP, heaterA.kP, heaterA.idx_kI, heaterA.kI, heaterA.idx_kD, heaterA.kD);
					encoder->setPosition(1);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;
				
				case 6:							// calibrate/heater PID/heater A/RESET
					LOAD_CAL(heaterA.idx_kP, &heaterA.kP, heaterA.idx_kI, &heaterA.kI, heaterA.idx_kD, &heaterA.kD);
					encoder->setPosition(1);
					menulevel[3] = 0;
					break;			

				default:
					menulevel[3] = 0;
					break;	
				}
				break;
			
			case 3: 							// calibrate/heater PID/Heater B
				switch (menulevel[3])
				{
				case 0:
					DISPLAY_SELECTOR(6);
					// run only once to save processing time
					if (display_static)
					{
						DISPLAY_CURSOR(1, 1);
						lcd.print("back");
						DISPLAY_CURSOR(2, 1);
						lcd.print("kP");
						DISPLAY_CURSOR(3, 1);
						lcd.print("kI");
						DISPLAY_CURSOR(4, 1);
						lcd.print("kD");
						DISPLAY_CURSOR(5, 1);
						lcd.print("SAVE");
						DISPLAY_CURSOR(6, 1);
						lcd.print("RESET");
						DISPLAY_CURSOR(2, 4);
						lcd.print(round(heaterB.kP));
						DISPLAY_CURSOR(3, 4);
						lcd.print(round(heaterB.kI));
						DISPLAY_CURSOR(4, 4);
						lcd.print(round(heaterB.kD));
						display_static = false;
					}
					break;
				
				case 1: 						// calibrate/heater PID/heater B/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:							// calibrate/heater PID/heater B/kP
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					DISPLAY_SETTER(&heaterB.kP, 1, 2, "kP");
					break;	

				case 3:							// calibrate/heater PID/heater B/kI
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					DISPLAY_SETTER(&heaterB.kI, 1, 3, "kI");
					break;	

				case 4:							// calibrate/heater PID/heater B/kD
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					DISPLAY_SETTER(&heaterB.kD, 1, 4, "kD");
					break;	

				case 5:							// calibrate/heater PID/heater B/SAVE
					SAVE_CAL(heaterB.idx_kP, heaterB.kP, heaterB.idx_kI, heaterB.kI, heaterB.idx_kD, heaterB.kD);
					encoder->setPosition(2);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;

				case 6:							// calibrate/heater PID/heater B/RESET
					LOAD_CAL(heaterB.idx_kP, &heaterB.kP, heaterB.idx_kI, &heaterB.kI, heaterB.idx_kD, &heaterB.kD);
					encoder->setPosition(2);
					menulevel[3] = 0;
					break;
				
				default:
					menulevel[3] = 0;
					break;	
				}
				break;

			case 4:								// calibrate/heater PID/Heater C
				switch (menulevel[3])
				{
				case 0:
					DISPLAY_SELECTOR(6);
					// run only once to save processing time
					if (display_static)
					{
						DISPLAY_CURSOR(1, 1);
						lcd.print("back");
						DISPLAY_CURSOR(2, 1);
						lcd.print("kP");
						DISPLAY_CURSOR(3, 1);
						lcd.print("kI");
						DISPLAY_CURSOR(4, 1);
						lcd.print("kD");
						DISPLAY_CURSOR(5, 1);
						lcd.print("SAVE");
						DISPLAY_CURSOR(6, 1);
						lcd.print("RESET");
						DISPLAY_CURSOR(2, 4);
						lcd.print(round(heaterC.kP));
						DISPLAY_CURSOR(3, 4);
						lcd.print(round(heaterC.kI));
						DISPLAY_CURSOR(4, 4);
						lcd.print(round(heaterC.kD));
						display_static = false;
					}
					break;
				
				case 1: 						// calibrate/heater PID/heater C/back
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;	

				case 2:	 						// calibrate/heater PID/heater C/kP
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(1);
						break;
					}
					DISPLAY_SETTER(&heaterC.kP, 1, 2, "kP");
					break;	

				case 3:	 						// calibrate/heater PID/heater C/kI
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(2);
						break;
					}
					DISPLAY_SETTER(&heaterC.kI, 1, 3, "kI");
					break;	

				case 4:	 						// calibrate/heater PID/heater C/kD
					if (!display_valuesetter)
					{
						menulevel[3] = 0;
						encoder->setPosition(3);
						break;
					}
					DISPLAY_SETTER(&heaterC.kD, 1, 4, "kD");
					break;	

				case 5: 						// calibrate/heater PID/heater C/SAVE
					SAVE_CAL(heaterC.idx_kP, heaterC.kP, heaterC.idx_kI, heaterC.kI, heaterC.idx_kD, heaterC.kD);
					encoder->setPosition(3);
					menulevel[2] = 0;
					menulevel[3] = 0;
					break;
				
				case 6: 						// calibrate/heater PID/heater C/RESET
					LOAD_CAL(heaterC.idx_kP, &heaterC.kP, heaterC.idx_kI, &heaterC.kI, heaterC.idx_kD, &heaterC.kD);
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

		case 3:									// calibrate/puller PID
			switch (menulevel[2])
			{
			case 0:								// calibrate/puller PID/
				DISPLAY_SELECTOR(6);
				// run only once to save processing time
				if (display_static)
				{
					DISPLAY_CURSOR(1, 1);
					lcd.print("back");
					DISPLAY_CURSOR(2, 1);
					lcd.print("kP");
					DISPLAY_CURSOR(3, 1);
					lcd.print("kI");
					DISPLAY_CURSOR(4, 1);
					lcd.print("kD");
					DISPLAY_CURSOR(5, 1);
					lcd.print("SAVE");
					DISPLAY_CURSOR(6, 1);
					lcd.print("RESET");
					DISPLAY_CURSOR(2, 4);
					lcd.print(round(puller.kP));
					DISPLAY_CURSOR(3, 4);
					lcd.print(round(puller.kI));
					DISPLAY_CURSOR(4, 4);
					lcd.print(round(puller.kD));
					display_static = false;
				}
				break;
			
			case 1: 							// calibrate/puller PID/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;	

			case 2:								// calibrate/puller PID/kP
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				DISPLAY_SETTER(&puller.kP, 100, 2, "kP");
				break;	

			case 3:								// calibrate/puller PID/kI
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				DISPLAY_SETTER(&puller.kI, 10, 3, "kI");
				break;	

			case 4:								// calibrate/puller PID/kD
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				DISPLAY_SETTER(&puller.kD, 10, 4, "kD");
				break;	

			case 5:								// calibrate/puller PID/SAVE
				SAVE_CAL(puller.idx_kP, puller.kP, puller.idx_kI, puller.kI, puller.idx_kD, puller.kD);
				encoder->setPosition(2);
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;
			
			case 6:								// calibrate/puller PID/RESET
				LOAD_CAL(puller.idx_kP, &puller.kP, puller.idx_kI, &puller.kI, puller.idx_kD, &puller.kD);
				encoder->setPosition(2);
				menulevel[2] = 0;
				break;

			default:
				menulevel[2] = 0;
				break;	
			}
			break;

		case 4:									// calibrate/size sensor
			switch (menulevel[2])
			{
			case 0:								// calibrate/size sensor/
				DISPLAY_SELECTOR(8);
				if (display_static)
				{
					DISPLAY_CURSOR(1, 1);
					lcd.print("back");
					DISPLAY_CURSOR(7, 1);
					lcd.print("SAVE");
					DISPLAY_CURSOR(8, 1);
					lcd.print("RESET");
					DISPLAY_CURSOR(2, 1);
					lcd.print(dia_size_cal[1]);
					DISPLAY_CURSOR(2, 6);
					lcd.print(int(dia_analog_val[1]));
					DISPLAY_CURSOR(3, 1);
					lcd.print(dia_size_cal[2]);
					DISPLAY_CURSOR(3, 6);
					lcd.print(int(dia_analog_val[2]));
					DISPLAY_CURSOR(4, 1);
					lcd.print(dia_size_cal[3]);
					DISPLAY_CURSOR(4, 6);
					lcd.print(int(dia_analog_val[3]));
					DISPLAY_CURSOR(6, 1);
					lcd.print(dia_size_cal[4]);
					DISPLAY_CURSOR(6, 6);
					lcd.print(int(dia_analog_val[4]));
					display_static = false;
				}
				break;

			case 1: 							// calibrate/size sensor/back
				menulevel[1] = 0;
				menulevel[2] = 0;
				break;	
			
			case 2:								// calibrate/size sensor/calibration pin 1
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(1);
					break;
				}
				DISPLAY_SETTER_SIZESENSOR(&dia_size_cal[1], &dia_analog_val[1], 0.01, 2, "Calibration pin 1");
				break;	

			case 3:								// calibrate/size sensor/calibration pin 2
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(2);
					break;
				}
				DISPLAY_SETTER_SIZESENSOR(&dia_size_cal[2], &dia_analog_val[2], 0.01, 3, "Calibration pin 2");
				break;	

			case 4:								// calibrate/size sensor/calibration pin 3
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(3);
					break;
				}
				DISPLAY_SETTER_SIZESENSOR(&dia_size_cal[3], &dia_analog_val[3], 0.01, 4, "Calibration pin 3");
				break;	

			case 6:								// calibrate/size sensor/calibration pin 4
				if (!display_valuesetter)
				{
					menulevel[2] = 0;
					encoder->setPosition(5);
					break;
				}
				DISPLAY_SETTER_SIZESENSOR(&dia_size_cal[4], &dia_analog_val[4], 0.01, 6, "Calibration pin 4");
				break;	

			case 7:								// calibrate/size sensor/SAVE
				SAVE_CAL_SIZESENSOR();
				menulevel[2] = 0;
				break;

			case 8:								// calibrate/size sensor/RESET
				LOAD_CAL_SIZESENSOR();
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

	case 3: 									// settings
		switch (menulevel[1])
		{
		case 0: 								// settings/
			DISPLAY_SELECTOR(4);
			// run only once to save processing time
			if (display_static)
			{
				DISPLAY_CURSOR(1, 1);
				lcd.print("back");
				DISPLAY_CURSOR(2, 1);
				lcd.print("Test mode");
				DISPLAY_CURSOR(3, 1);
				lcd.print("Serial logging");
				DISPLAY_CURSOR(4, 1);
				lcd.print("Restore defaults");

				CHECK_MARK(TEST_MODE, 2);
				CHECK_MARK(SERIAL_LOGGING, 3);

				display_static = false;
			}
			break;

		case 1: 								// settings/back
			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		case 2: 								// settings/Test mode
			TEST_MODE = !TEST_MODE;
			display_static = true;
			menulevel[1] = 0;
			break;

		case 3: 								// settings/serial logging
			SERIAL_LOGGING = !SERIAL_LOGGING;
			display_static = true;
			menulevel[1] = 0;
			break;

		case 4: 								// settings/restore defaults
			encoder->setPosition(2);

			// defaults, EDIT HERE IF YOU WANT TO CHANGE DEFAULTS
			heaterA.Set_setpoint(0);
			heaterB.Set_setpoint(0);
			heaterC.Set_setpoint(0);
			puller.Set_setpoint(1.75);
			SAVE_SETPOINT();

			heaterA.Set_kPID(200, 7, 1, REVERSE);
			heaterB.Set_kPID(200, 7, 1, REVERSE);
			heaterC.Set_kPID(200, 7, 1, REVERSE);
			puller.Set_kPID(100000, 5000, 500, DIRECT);
			SAVE_CAL_ALL();

			float dia_analog_val_def[6] = {0, 600, 622, 692, 883, 1023};
			float dia_size_cal_def[6] = {3, 2.00, 1.75, 1.60, 1.20, 0};
			for (uint8_t i = 0; i < 6; i++)
			{
				EEPROM.put(dia_analog_val_idx[i], dia_analog_val_def[i]);
				EEPROM.put(dia_size_cal_idx[i], dia_size_cal_def[i]);
			}
			LOAD_CAL_ALL();

			menulevel[0] = 0;
			menulevel[1] = 0;
			break;

		default:
			menulevel[1] = 0;
			break;
		}
		break;

	case 4:										// about
		switch (menulevel[1])
		{
		case 0:									// about/
			if (display_static)
			{
				DISPLAY_CURSOR(1, 1);
				lcd.print("BSME 4-2 batch 2022");
				DISPLAY_CURSOR(2, 1);
				lcd.print("Denisse");
				DISPLAY_CURSOR(3, 1);
				lcd.print("bitsuki");
				DISPLAY_CURSOR(4, 2);
				lcd.print("D'AEms");
				DISPLAY_CURSOR(6, 0);
				lcd.print("densanity");
				DISPLAY_CURSOR(7, 1);
				lcd.print("RealED");
				DISPLAY_CURSOR(8, 1);
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

// ISRs
void RESET_TIMER()
{
	PORTA &= !B11100000;
	zero_cross = true;
	TCNT4 = 0;
	TCNT5 = 0;
}

void CHECK_POSITION()
{
	encoder->tick(); 							// just call tick() to check the state.
}

void BUTTON_PRESSED()
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

ISR(TIMER3_COMPA_vect)
{
	// motor_pulsecount++;
}

// turns on firing pulse for heater 1
ISR(TIMER4_COMPA_vect)
{
	if (zero_cross)
	{
		PORTA |= B10000000; 					// turns pin 29 on
	}
}

// turns on firing pulse for heater 2
ISR(TIMER4_COMPB_vect)
{
	if (zero_cross)
	{
		PORTA |= B01000000; 					// turns pin 28 on
	}
}

// turns on firing pulse for heater 3
ISR(TIMER4_COMPC_vect)
{
	if (zero_cross)
	{
		PORTA |= B00100000; 					// turns pin 27 on
	}
}

// turns off firing pulse for heater 1,2,3
ISR(TIMER5_COMPA_vect)
{
	PORTA &= !B11100000; 						// turns pin 29, 28, 27 off
	zero_cross = false;

	if (TEST_MODE)
	{
		RESET_TIMER();
	}
}