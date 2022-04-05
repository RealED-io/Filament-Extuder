#ifndef PID_AC_Heater_h
#define PID_AC_Heater_h

#include <Arduino.h>
#include <PID_v1.h>

class ACHeater{
    public:
        ACHeater(double, double, double,
                    double, double, double);
        double kP;
        double kI;
        double kD;

        double set_temp;        //PID setpoint  
        double temp;            //PID input
        double pulse_delay;     //PID output

        void PID_compute();

    private:

};

#endif