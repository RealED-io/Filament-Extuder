#include <Arduino.h>
#include "ACHeater.h"


ACHeater::ACHeater(double set_temp, double constP, double constI, double constD, bool direction)
{
    Set_Temp = set_temp;
    kP = constP;
    kI = constI;
    kD = constD;
    PID_Direction = direction;
}

void ACHeater::Compute(unsigned int Compute_Delay)      //Compute_Delay unit is in ms
{
    double Error, PID_value;

    Error = Set_Temp - Temp;
    //disable Integral when Error is high
    if(Error > 30)
    {
        PID_I = 0;
    }

    //individually compute for P, I, and D
    PID_P = kP * Error;
    PID_I = PID_I + (kI * Error);
    PID_D = kD * ( (Error - Error_Previous) / (Compute_Delay * 1000) );

    //PID total
    PID_value = PID_P + PID_I + PID_D;
    
    //set PID value only within the specified range
    if(PID_value > Delay_Max)
    {
        PID_value = Delay_Max;
    }
    else if (PID_value < Delay_Min)
    {
        PID_value = Delay_Min;
    }
    
    //reversed direction for heater control
    if(PID_Direction)
    {
        Pulse_Delay = (Delay_Max + 1) - PID_value;        
    }
    else
    {
        Pulse_Delay = PID_value;
    }

}
