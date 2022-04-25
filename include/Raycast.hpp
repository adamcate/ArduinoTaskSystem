#pragma once
#include "BitMatrix.hpp"
#include "vec.h"

Vec3 castRay(bitMatrix& grid, int gridScale, Vec2 posInitial, Vec2 posFinal);

void castRayFan(bitMatrix& matrix, Vec3 * outputs, Vec2 origin, float len, float initialAngle, int noRays, float angleSpread);