#ifndef ACHeater_h
#define ACHeater_h

#include <Arduino.h>
#include <PID_v1.h>

class ACHeater{
    public:
        ACHeater(double, double, double,
                    double, double, double, unsigned int);


        double Temp;            //PID input
        double Pulse_Delay;     //PID output
        double Set_Temp;        //PID setpoint  
        
        double kP;
        double kI;
        double kD;

        unsigned int Max_Pulse_Delay;

        void PID_compute();

    private:

};

#endif