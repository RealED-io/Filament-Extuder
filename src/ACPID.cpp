#include <Arduino.h>
#include "ACPID.h"


ACPID::ACPID(double set, double constP, double constI, double constD, bool direction)
{
    Setpoint = set;
    kP = constP;
    kI = constI;
    kD = constD;
    PID_Direction = direction;
}

void ACPID::Compute(unsigned int Compute_Delay)      //Compute_Delay unit is in ms
{
    double Error, PID_value;

    Error = (Setpoint - Input);
    // //disable Integral when Error is high
    // if(Error > 30)
    // {
    //     PID_I = 0;
    // }

    //Reset PID_I after passing set
    if((Error < 0) && (Error_Previous > 0)){
        PID_I = 0;
    };

    //individually compute for P, I, and D
    PID_P = kP * Error;
    PID_I = PID_I + (kI * Error);
    PID_D = kD * ( (Error - Error_Previous) / (Compute_Delay) );

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
        Pulse_Delay = (Delay_Max + Delay_Min) - PID_value;        
    }
    else
    {
        Pulse_Delay = PID_value;
    }

    Error_Previous = Error;
    
    // Serial.println(PID_P);
    // Serial.println(PID_I);
    // Serial.println(PID_D);
    // Serial.println(PID_value);
    // Serial.println(Pulse_Delay);
    // Serial.println();


}

void ACPID::Range(unsigned int min, unsigned int max)
{
    Delay_Min = min;
    Delay_Max = max;
}
