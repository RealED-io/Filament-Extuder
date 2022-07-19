#ifndef ACPID_h
#define ACPID_h

#define DIRECT true
#define REVERSE false

#include <Arduino.h>


class ACPID{
    public:
        ACPID(double, unsigned int, unsigned int, unsigned int, bool);

        bool PID_Direction;     //true for reversed, false for direct
        bool PID_I_reset = true;
        double Input;            //PID input
        double Pulse_Delay;     //PID output
        double Setpoint;        //PID setpoint  
        
        double kP;
        double kI;
        double kD;

        unsigned int idx_kP;
        unsigned int idx_kI;
        unsigned int idx_kD;

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
        void Set_kPID(double, double, double);
        // void Set_EEPROM_idx(unsigned int, unsigned int, unsigned int);
};

#endif