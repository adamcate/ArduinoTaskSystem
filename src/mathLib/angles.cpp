#include "mathLib\angles.hpp"
#include "types.h"

f32 angleDiff(f32 angleInitial, f32 angleFinal)
{
	f32 a = angleFinal - angleInitial;
	a = MOD_U(a + 180.f, 360.f) - 180.f;
	return a;
}

f32 getAngle(f32 x, f32 y) 
{
	f32 angle = atan2f(y,x) * RAD_TO_DEG;
	if (angle >= 0.f) return angle;
	return 360.f + angle;
}
