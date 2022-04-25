#ifndef DRIVING_H
#define DRIVING_H

#include "types.h"

// function to control the motors based on sign & magnitude
void drive(s16 left, s16 right);

// linearly interpolate between start 
// & end based on t's fraction of 1
f32 lerpf(f32 start, f32 end, f32 t);


#endif