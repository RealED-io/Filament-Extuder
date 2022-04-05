#include <Arduino.h>
#include "ACHeater.h"


ACHeater::ACHeater(double temp, double pulse_delay, double set_temp,
                    double constP, double constI, double constD, unsigned int max_pulse_delay)
{
    Temp = temp;
    Pulse_Delay = pulse_delay;
    Set_Temp = set_temp;
    constP = kP;
    constI = kI;
    constD = kD;
    Max_Pulse_Delay = max_pulse_delay;

    PID myACHeater(&Temp, &Pulse_Delay, &Set_Temp, kP, kI, kD, REVERSE);
    myACHeater.SetOutputLimits(1, Max_Pulse_Delay);
    myACHeater.SetMode(AUTOMATIC);
    
}

void ACHeater::PID_compute()
{
    //myACHeater.Compute();
}
