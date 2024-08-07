/*
UN178 dual channel 100A brushed motor driver
Also known as "big green motor controller"

Original in 4/8/2019 by Arsh Chauhan
*/
#ifndef NANOSLOT_FIRMWARE_UN178_MOTOR_H
#define NANOSLOT_FIRMWARE_UN178_MOTOR_H

#include <Arduino.h>


// One motor channel, half a UN178 driver board
class un178_motor_single_t
{
    //Needs 2 digital pins and 1 PWM pin

    uint8_t pwm_;
    uint8_t dir_1_, dir_2_;

public:

    un178_motor_single_t(uint8_t pwm, uint8_t dir_1, uint8_t dir_2 ):
    pwm_(pwm), dir_1_(dir_1), dir_2_(dir_2){
        stop();
    }

    // Configure our pins as outputs
    void set_pin_modes()
    {
        pinMode(pwm_,OUTPUT);
        pinMode(dir_1_,OUTPUT);
        pinMode(dir_2_,OUTPUT);
        stop();
    }

    // Drive this motor using raw PWM and direction (0 or 1) values 
    void drive (uint8_t pwm, uint8_t dir_1, uint8_t dir_2)const
    {
        if (pwm >= 255 )
            pwm = 254; //UN178 stops working if it gets 255 PWM
        digitalWrite(dir_1_,dir_1);
        digitalWrite(dir_2_,dir_2);
        analogWrite(pwm_,pwm);
    }

    inline void drive_green(uint8_t pwm)const
    {
        drive(pwm,1,0);
    }

    inline void drive_red(uint8_t pwm)const
    {
        drive(pwm,0,1);
    }

    inline void stop()const
    {
        drive(0,0,0);
    }
};


/*
Scale speed from -100 .. +100
to -254 .. +254 
(Can't send full 255, the UN178 will shut off)
*/
int16_t power_percent_to_pwm(int8_t speed)
{
    if (speed>=100) return 254;
    if (speed<=-100) return -254;
    int16_t pwm = (int16_t(speed)*254)/100; // floor(254*(double(speed)/100));
    return pwm;
}

void send_motor_power(const un178_motor_single_t &motor, int8_t speed)
{
    int16_t pwm = power_percent_to_pwm(speed);
    if(pwm==0)
      motor.stop();
    else if(pwm>0)
      motor.drive_green(pwm);
    else // pwm<0
      motor.drive_red(-pwm);
}

#ifdef NANOSLOT_COMMAND_MY
/* Hardware-connected motor drivers, UN178 green brushed boards,
   using our breakout board.  
   These pins number the motors [0] through [3] from left to right.
*/
un178_motor_single_t hardware_motor[NANOSLOT_COMMAND_MY::n_motors]={
    un178_motor_single_t(6,5,7),
    un178_motor_single_t(3,2,4),
    un178_motor_single_t(10,9,8),
    un178_motor_single_t(11,12,A0),
};
#endif

#endif
