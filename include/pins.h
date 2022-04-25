#ifndef PINS_H
#define PINS_H


#define MEGA


#ifdef MEGA 
#define BRAKEVCC 0

#define PWM_1 45    //5
#define PWM_2 46    //6

#define CW_1  27
#define CCW_1 28

#define CW_2 25
#define CCW_2 26

#define BRAKEGND 3

#define EN_1 A8
#define EN_2 A9

#define CURR_SENS_1 A2
#define CURR_SENS_2 A3

#endif

#ifdef MEGA
#define INIT_MOTOR_PINS(){/*macro shortcut for setting all nessesary pinmodes*/\
pinMode(PWM_1,OUTPUT);      \
pinMode(PWM_2,OUTPUT);      \
pinMode(CW_1,OUTPUT);       \
pinMode(CCW_1,OUTPUT);      \
pinMode(CW_2,OUTPUT);       \
pinMode(CCW_2,OUTPUT);      \
pinMode(EN_1,OUTPUT);\
pinMode(EN_2,OUTPUT);\
pinMode(CURR_SENS_1,OUTPUT);\
pinMode(CURR_SENS_2,OUTPUT);\
}                           
#endif

#endif