#ifndef ACPID_h
#define ACPID_h

#define DIRECT true
#define REVERSE false

#include <Arduino.h>


class ACPID{
    public:
        ACPID(bool);
        // ACPID(float, unsigned int, unsigned int, unsigned int, bool);

        bool PID_Direction;     //true for reversed, false for direct
        bool PID_I_reset = true;
        bool PID_I_limit = true;
        float Input;            //PID input
        float Pulse_Delay;     //PID output
        float Setpoint;        //PID setpoint  
        
        float kP;
        float kI;
        float kD;

        unsigned int idx_Set;
        unsigned int idx_kP;
        unsigned int idx_kI;
        unsigned int idx_kD;

        float PID_P;
        float PID_I = 0;
        float PID_D;
        float PID_value;

        float PID_I_resetatError = 60000;
        float Error_Previous;

        unsigned int Delay_Min;
        unsigned int Delay_Max;

        void Set_setpoint(float);
        void Compute(unsigned int);
        void Range(unsigned int, unsigned int);
        void Set_kPID(float, float, float, bool);
        void EEPROM_idx(unsigned int, unsigned int, unsigned int, unsigned int);
};

#endif