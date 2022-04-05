#ifndef ACHeater_h
#define ACHeater_h

#include <Arduino.h>


class ACHeater{
    public:
        ACHeater(double, double, double,
                    double, double, double);


        double Temp;            //PID input
        double Pulse_Delay;     //PID output
        double Set_Temp;        //PID setpoint  
        
        double kP;
        double kI;
        double kD;
};

#endif