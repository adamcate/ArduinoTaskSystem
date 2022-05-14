#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// READ BEFORE BEGINNING

// Arduino does not use standard type sizes found on modern computers
// so these names are more descriptive.
// (u is unsigned, s is signed, the number represents the number of bits)

// You should avoid using larger data types when possible, since they take up much more memory
// and operations are MUCH slower on them. Be careful when using unsigned types, since they cannot represent
// negative values. When in doubt, use signed types unless you know what you're doing



#define u8 unsigned char    // values range from 0 - +255 (8 bits)
#define s8 char             // values range from -128 - +127

#define u16 unsigned int    // values range from 0 - +65,536 (16 bits, 2^16 values)
#define s16 int             // values range from -32,768 - +32,767

#define u32 unsigned long   // values range from 0 - +4,294,967,295
#define s32 long            // values range from -2,147,483,648 - +2,147,483,647


#define u64 uint64_t        // values range from 0 - +18,446,744,073,709,551,615
#define s64 int64_t         // values range from -9,223,372,036,854,775,808 - +9,223,372,036,854,775,807

#define f32 float  // floating point numbers are always 32 bit on Arduino, double and float are identical


// NOTE: floating point types (aka decimal numbers) are always 32 bit on Arduino. There is no difference between float and double
// unlike many other systems

// The Arduino DOES NOT NATIVELY SUPPORT DECIMAL FORMATS. You can still use them, but the system uses
// trickery to get them to work. Therefore, doing operations with floats is far slower than integer types

// In the vast majority of cases, this doesn't matter, but for programs that require greater performance, you should limit operations with floats
// as much as possible.



// (If you want to represent 3 decimal points of precision, but want higher performance you could use an integer. 
// for example: 1.005 could be represented as 1005, just remember to take the extra factor into account!)


#endif