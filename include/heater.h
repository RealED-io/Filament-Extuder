#ifndef PID_AC_Heater_h
#define PID_AC_Heater_h

#include <Arduino.h>

class ACHeater{
    public:
        ACHeater();
        uint16_t kP;
        uint16_t kI;
        uint16_t kD;
        uint16_t P;
        uint16_t I;
        uint16_t D;  
        int16_t set_temp;      
        int16_t temp;

        uint16_t PID_compute();

    private:

};

#endif