#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// Arduino does not use standard type sizes found on x86, x64, or AMD64,
// so these names are more descriptive.
// (u is unsigned, s is signed, the number represents the number of bits)

#define u8 unsigned char
#define s8 char

#define u16 unsigned int
#define s16 int

#define u32 unsigned long
#define s32 long

#define u64 uint64_t
#define s64 int64_t


#define f32 float  // floating point numbers are always 32 bit on Arduino, double and float are identical


#endif