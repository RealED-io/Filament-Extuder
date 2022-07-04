#include <Arduino.h>
#include "ACPID.h"

#ifndef LOGGING
  #define LOGGING 0
  // 0 NONE
  // 1 FATAL
  // 2 ERROR
  // 3 WARNING
  // 4 INFO
  // 5 DEBUG
  // 6 TRACE
#endif

ACPID::ACPID(double set, double constP, double constI, double constD, bool direction)
{
    Setpoint = set;

    PID_Direction = direction;

    if(PID_Direction)       // DIRECT
    {
        kP = constP;
        kI = constI;
        kD = constD;
    }
    else                    // REVERSE
    {
        kP = (0 - constP);
        kI = (0 - constI);
        kD = (0 - constD);
    }
}

void ACPID::Range(unsigned int min, unsigned int max)
{
    Delay_Min = min;
    Delay_Max = max;
    
    if(PID_Direction)       // DIRECT
    {
        PID_I = Delay_Min;
    }
    else                    // REVERSE
    {
        PID_I = Delay_Max;
    }
}

void ACPID::Compute(unsigned int Compute_Delay)      //Compute_Delay unit is in ms
{
    double Error;

    Error = (Setpoint - Input);
    
    //disable Integral when Error is high
    if(Error > PID_I_disableatError)
    {
        PID_I = 0;
    }

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
        Pulse_Delay = Delay_Max;
    }
    else if (PID_value < Delay_Min)
    {
        Pulse_Delay = Delay_Min;
    }
    else
    {
        Pulse_Delay = PID_value;
    }

    //reversed direction for heater control
    // if(PID_Direction)
    // {
    //     Pulse_Delay = (Delay_Max + Delay_Min) - PID_value;        
    // }
    // else
    // {
    //     Pulse_Delay = PID_value;
    // }

    Error_Previous = Error;
    
    #if LOGGING >= 5
        // Serial.println(PID_P);
        // Serial.println(PID_I);
        // Serial.println(PID_D);
        Serial.println(PID_value);
        Serial.println(Pulse_Delay);
        // Serial.println();
    #endif

}


