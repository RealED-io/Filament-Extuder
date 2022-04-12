#ifndef ACHeater_h
#define ACHeater_h

#include <Arduino.h>


class ACHeater{
    public:
        ACHeater(double, double, double, double, bool);

        bool PID_Direction;     //true for reversed, false for direct
        double Temp;            //PID input
        double Pulse_Delay;     //PID output
        double Set_Temp;        //PID setpoint  
        
        double kP;
        double kI;
        double kD;

        double PID_P;
        double PID_I = 0;
        double PID_D;

        double Error_Previous;

        unsigned int Delay_Min;
        unsigned int Delay_Max;

        void Compute(unsigned int);
        void Range(unsigned int, unsigned int);
};

#endif