#ifndef ANGLES_H
#define ANGLES_H

#include "types.h"
#include "vec.h"

f32 angleDiff(f32 init, f32 fin);
f32 getAngle(f32 x, f32 y);

#define MOD_U(a,n) fmodf(fmod(a,n)+n,n)
#endif