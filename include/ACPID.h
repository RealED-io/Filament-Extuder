#ifndef ACPID_h
#define ACPID_h

#define DIRECT true
#define REVERSE false

#include <Arduino.h>


class ACPID{
    public:
        ACPID(double, double, double, double, bool);

        bool PID_Direction;     //true for reversed, false for direct
        double Input;            //PID input
        double Pulse_Delay;     //PID output
        double Setpoint;        //PID setpoint  
        
        double kP;
        double kI;
        double kD;

        double PID_P;
        double PID_I = 0;
        double PID_D;
        double PID_value;

        double PID_I_disableatError = 60000;
        double Error_Previous;

        unsigned int Delay_Min;
        unsigned int Delay_Max;

        void Compute(unsigned int);
        void Range(unsigned int, unsigned int);
};

#endif