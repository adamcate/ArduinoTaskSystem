#pragma once
#include "types.h"
/*
Code to reduce the memory footprint of boolean-like matricies by a factor of 8.
*/


struct bitMatrix
{
	s16 width = 0, height = 0;			//defaults to 0 until initialized
	u8* array = nullptr;			//bit array contents (pointer to null until initialized)
	u32 arrsize = 8ul;					

	bitMatrix();						//default constructor
	bitMatrix(s16 width, s16 height);	//specify height in constructor
	~bitMatrix();						//free program memory when out of scope
};


bool initializeBitMatrix(bitMatrix& matrix, s16 width, s16 height, u8 contents[]);	//initialize the bitMatrix with a set of contents
void initializeBitMatrix(bitMatrix& matrix, s16 width, s16 height);						//initialize the bitMatrix as empty (0s)

bool getBitMatrixElement(bitMatrix& matrix, s16 x, s16 y);								//return the value at a given x and y index of the matrix
void setBitMatrixElement(bitMatrix& matrix, s16 x, s16 y, bool value);					//set the bit at x and y

void printBitMatrix(bitMatrix& matrix);													//print a graphical representation of the matrix