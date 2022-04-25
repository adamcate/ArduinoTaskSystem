#include "driving.hpp"
#include "pins.h"

// TODO check to see if updated code for this actually switches wheels properly
// function to control the motors based on sign & magnitude


void drive(s16 left, s16 right){
    if(left > 255) left = 255;
    else if (left < -255) left = -255;

    if(right > 255) right = 255;
    else if (right < -255) right = -255;

    digitalWrite(CW_1,right <= 0 ? HIGH : LOW);
    digitalWrite(CCW_1,right > 0 ? HIGH : LOW);
    analogWrite(PWM_1,abs(right));

    digitalWrite(CW_2,left >= 0 ? HIGH : LOW);
    digitalWrite(CCW_2,left < 0 ? HIGH : LOW);
    analogWrite(PWM_2,abs(left));
}

// linearly interpolate between start 
// & end based on t's fraction of 1
f32 lerpf(f32 start, f32 end, f32 t){
    if(t > 1.f) t = 1.f;
    else if(t < 0.f) t = 0.f;
    return start + (end - start) * t;
}