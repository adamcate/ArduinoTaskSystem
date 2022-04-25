#include "BitMatrix.hpp"
#include <Arduino.h>
#include "Driving.hpp"

bitMatrix::bitMatrix()
{

}

bitMatrix::bitMatrix(s16 width, s16 height)
{
	initializeBitMatrix(*this, width, height);
}

bitMatrix::~bitMatrix()	// free matrix array memory when no longer used
{
	delete[] array;
	array = nullptr;
}

bool initializeBitMatrix(bitMatrix& matrix, s16 width, s16 height, u8 contents[])
{
	matrix.width = width;
	matrix.height = height;

	if (matrix.array != nullptr)					// if the memory in array is not free, delete the contents and free it
	{												//so it can be modified
		delete[] matrix.array;
		matrix.array = nullptr;
	}

	matrix.arrsize = ceil(f32(width * height)/ 8);	// set the number of elements to the amount nessesary
	matrix.array = new u8[matrix.arrsize];		//to hold the data

	for (u32 i = 0; i < matrix.arrsize; ++i)		// set the array equal to the contents
	{
		matrix.array[i] = contents[i];
	}
	return true;
}

void initializeBitMatrix(bitMatrix& matrix, s16 width, s16 height)
{
	matrix.width = width;
	matrix.height = height;

	if (matrix.array != nullptr)
	{											
		delete[] matrix.array;
		matrix.array = nullptr;
	}

	matrix.arrsize = ceil(f32(width * height) / 8.f);
	matrix.array = new u8[matrix.arrsize];

	for (u32 i = 0; i < matrix.arrsize; ++i)		// set the array equal to the contents
	{
		matrix.array[i] = (u8)0x0;			// initialize as all zeros
	}
}


bool getBitMatrixElement(bitMatrix& matrix, s16 x, s16 y)
{
	if (x > matrix.width || y > matrix.height || x < 0 || y < 0)		// return false (0) if out of bounds
		return false;

	u32 fullIndex = y * matrix.width + x;			// fullIndex is the nth bit of the data
	u32 arrIndex = fullIndex / 8;					// integer division (truncation) to get the nth index of matrix.array
	u16 bitIndex = fullIndex % 8;					// the sub index bit of the nth index of the char in matrix.array

	u8 bitSet = matrix.array[arrIndex];		// the byte that contains the desired value

	return ((bitSet >> bitIndex) & 0x01) == 0x01;	// shift bitSet by the sub index and return as boolean
}

void setBitMatrixElement(bitMatrix& matrix, s16 x, s16 y, bool value)
{
	if (x > matrix.width || y > matrix.height || y < 0 || x < 0)		// exit if out of bounds
		return;

	u32 fullIndex = y * matrix.width + x;			// fullIndex is the nth bit of the data
	u32 arrIndex = fullIndex / 8;					// integer division (truncation) to get the nth index of matrix.array
	u8 bitIndex = fullIndex % 8;					// the sub index bit of the nth index of the char in matrix.array

	u8 inputSet = ((u8)0x1) << bitIndex;
	
	if (value) {
		matrix.array[arrIndex] = matrix.array[arrIndex] | inputSet;
		return;
	}
	inputSet = ~inputSet;
	matrix.array[arrIndex] = matrix.array[arrIndex] & inputSet;
}


void printBitMatrix(bitMatrix& matrix)
{
	for (s16 i = 0; i < matrix.height; ++i)
	{
		for (s16 j = 0; j < matrix.width; ++j)
		{
			Serial.print(getBitMatrixElement(matrix, j, i));
		}
		Serial.println();
	}
	Serial.println();
}
