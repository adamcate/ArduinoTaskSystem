// vec.h
#ifndef VEC_H
#define VEC_H

#include "types.h"


struct Vec2
{
    float i = 0, j = 0;

    Vec2();
    Vec2(const Vec2& arg);
    Vec2(f32 x, f32 y);

    Vec2& operator=(const Vec2& second);

    Vec2 operator+(const Vec2& second) const;
    Vec2 operator-(const Vec2& second) const;

    f32 operator*(const Vec2& second) const;  // dot product
    Vec2 operator*(const f32 second) const;
};


struct Vec3
{
    f32 i = 0, j = 0, k = 0;

    Vec3();
    Vec3(const Vec3& arg);
    Vec3(f32 x, f32 y, f32 z);

    Vec3& operator=(const Vec3& second);

    Vec3 operator+(const Vec3& second) const;
    Vec3 operator-(const Vec3& second) const;

    f32 operator*(const Vec3& second) const;  // dot product
    Vec3 operator*(const f32 second) const;
};


f32 magnitude(Vec2 vec);
f32 magnitude(Vec3 vec);


Vec3 crossProduct(Vec3 first, Vec3 second);

Vec2 proj(Vec2 u, Vec2 v);
Vec3 proj(Vec3 u, Vec3 v);


Vec2 normalize(Vec2 vec);
Vec3 normalize(Vec3 vec);

f32 AngleFromHorizontal(Vec2 vec);

#endif