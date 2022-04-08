#include <Arduino.h>
#include "ACHeater.h"


ACHeater::ACHeater(double temp, double pulse_delay, double set_temp,
                    double constP, double constI, double constD)
{
    Temp = temp;
    Pulse_Delay = pulse_delay;
    Set_Temp = set_temp;
    kP = constP;
    kI = constI;
    kD = constD;
 
}

