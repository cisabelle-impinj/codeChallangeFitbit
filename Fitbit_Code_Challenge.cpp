// Fitbit_Code_Challenge.cpp : Defines the entry point for the console application.
//

/*
Christopher J. Isabelle
christopher.j.isabelle@gmail.com
619-852-4894

Fitbit Senior Firmware Engineer Practical Coding Challenge

Design assumptions and considerations:

A quick search on the internet shows a number of low cost, 3-Axis, 12-bit digital accelerometers with 32 value FIFO and I2C.
The ARM Cortex M series has built in support for I2C.  This device model has functionality that is consistent with the original problem statement.

This project is a device simulator (using file I/O) and the middleware to pre-process 32 sample 12-bit packed accelerometer values for processing by upper algorithms for motion, transient, tap and orientation detection.

The middleware functions have been optimized to run on target.

The file I/O functions are for simulation only and have not been optimized.

For ease of review all code and comments are contained in a single file.

Original problem statement is appended to the end of this file.
*/

#include "stdafx.h"
#include <math.h>
#include <stdio.h>

//create C99 exact width type defs given platform assupmtions as stated
typedef  unsigned char    uint8_t;     //  8-bit unsigned value
typedef  char             int8_t;      //  8-bit signed value
typedef  unsigned short   uint16_t;    // 16-bit unsigned value
typedef  short            int16_t;     // 16-bit signed value
typedef  unsigned long    uint32_t;    // 32-bit unsigned value
typedef  long             int32_t;     // 32 bit signed value

//type to hold 32 12-bit accelerometer values
//unpacked for ease of use by upper layer functions
//minimal overhead by using 16-bit storage
typedef struct {
	uint16_t numVals;	
	uint16_t val[32];	
}accelerometerBuf_t;

uint32_t bytes_read_from_file = 0;	//used to track num bytes read this call

//****************************************************************
// Function Name: getAccelerometerFIFOFromFile
//
// Description: retrieves up to 32 12-bit packed FIFO from an accelerometer simulation file
//
// parameters:	packedFIFO (output) - 48 byte array to hold 32 12-bit accelerometer values
//				numBytes (output) - number of bytes retrieved from the simulation file
//
// return val:	-1 > invalid devID or bad file/path name
//				 0 > data remaining in the simulation file
//               1 > no data remaining in the simulation file
//****************************************************************
int32_t getAccelerometerFIFOFromFile(char *fileName, uint8_t *packedFIFO, uint32_t *numBytes)
{
	FILE *pFile;
	uint32_t fileSize;
	uint32_t rtnVal = 0;

	pFile = fopen(fileName, "rb");
	if (pFile == 0x00)
		return(-1);

	//find how many bytes in the simulation file
	fseek(pFile, 0, SEEK_END);
	fileSize = ftell(pFile);

	//determine how many bytes to read this iteration
	*numBytes = (fileSize - bytes_read_from_file);
	if (*numBytes > 48)
		*numBytes = 48;

	//position file pointer and read bytes
	fseek(pFile, bytes_read_from_file, SEEK_SET);
	int ii = fread(packedFIFO, 1, *numBytes, pFile);

	//update control variables
	bytes_read_from_file += *numBytes;
	if (bytes_read_from_file>=fileSize)
	{
		bytes_read_from_file = 0;
		rtnVal = 1;
	}
	fclose(pFile);

	return(rtnVal);
}

//****************************************************************
// Function Name: outputData
//
// Description: retrieves up to 32 12-bit packed FIFO from an accelerometer simulation file
//
// parameters:	accelerometerBuf (input) - struct holding last 32 values read
//				accelerometerMax (input) - struct holding 32 max values sorted with max at index 0
//
//****************************************************************
void outputData(char *fileName, accelerometerBuf_t * accelerometerBuf, accelerometerBuf_t * accelerometerMax)
{

	FILE *pFile;
	int32_t iteration;

	pFile = fopen(fileName, "w");

	if (pFile == 0x00)
		return;

	fprintf(pFile, "--Sorted Max 32 Values--\n");
	for (iteration = 0; iteration < accelerometerMax->numVals; iteration++)
		fprintf(pFile, "%i\n", accelerometerMax->val[accelerometerMax->numVals-iteration-1]);

	fprintf(pFile, "--Last 32 Values--\n");
	for (iteration = 0; iteration < accelerometerBuf->numVals; iteration++)
		fprintf(pFile, "%i\n", accelerometerBuf->val[iteration]);

	fclose(pFile);

}

//****************************************************************
// Function Name: unpackAccelerometerFIFO
//
// Description: unpackets upto 32 12-bit accelerometer values
//				designed to model a 48 byte read of 32 12-bit packed accelerometer values
//
// parameters:	packedFIFO (input) - packed data from device of simulation file
//				numBytes (input) - number of bytes
//				accelerometerFIFO (output) - holds 32 unpacked 12-bit accelerometer values
//
//****************************************************************

void unpackAccelerometerFIFO(uint8_t *packedFIFO, uint32_t numBytes, accelerometerBuf_t *accelerometerFIFO)
{
	uint8_t currentByte;	//pointer to current byte in packedFIFO

	for (accelerometerFIFO->numVals = 0, currentByte = 0; currentByte < numBytes; currentByte++)
	{
		if (accelerometerFIFO->numVals & 1)
		{
			//accelVal[11:8] = "previousByte[3:0]", accelVal[7:0] = currentByte[7:0]
			accelerometerFIFO->val[accelerometerFIFO->numVals] = packedFIFO[currentByte - 1] & 0x0f << 8 | packedFIFO[currentByte];
			accelerometerFIFO->numVals++;
		}
		else
		{
			//accelVal[11:4] = currentByte[7:0], accelVal[3:0] = "nextByte[7:4]"
			accelerometerFIFO->val[accelerometerFIFO->numVals] = packedFIFO[currentByte] << 4 | (packedFIFO[currentByte + 1] & 0xf0) >> 4;
			accelerometerFIFO->numVals++;
			currentByte ++ ; //skip another byte for next iteration
		}
	}
	return;
}

//****************************************************************
// Function Name: processAccelerometerBuf
//
// Description: adds new accelerometer data into accelerometerBuf
//
// parameters:	accelerometerFIFO (input) - new accelerometer values
//				accelerometerBuf (output) - holds latest 32 unpacked 12-bit accelerometer values
//
//****************************************************************
void processAccelerometerBuf(accelerometerBuf_t *accelerometerFIFO, accelerometerBuf_t *accelerometerBuf)
{
	int8_t iBuf;
	int8_t iFIFO=0;
	int16_t numValsFromBuf;	//number of 12-bit values to be used from existing accelerometerBuf

	if (accelerometerBuf->numVals < (32 - accelerometerFIFO->numVals))
		numValsFromBuf = accelerometerBuf->numVals;
	else
		numValsFromBuf = 32 - accelerometerFIFO->numVals;

	for (iBuf = 0; iBuf < (numValsFromBuf + accelerometerFIFO->numVals); iBuf++)
	{
		if (iBuf < numValsFromBuf)
		{
			//fill available values from accelerometerBuf
			accelerometerBuf->val[iBuf] = accelerometerBuf->val[accelerometerBuf->numVals - numValsFromBuf + iBuf];
		}
		else
		{
			//fill new values from accelerometerFIFO
			accelerometerBuf->val[iBuf] = accelerometerFIFO->val[iFIFO];
			iFIFO++;
		}
	}
	accelerometerBuf->numVals = iBuf;
}

//****************************************************************
// Function Name: processAccelerometerMax
//
// Description: adds new accelerometer max reading into accelerometerMax
//
// parameters:	accelerometerFIFO (input) - new accelerometer values
//				accelerometerBuf (output) - holds the 32 largest unpacked 12-bit accelerometer values
//
//****************************************************************
void processAccelerometerMax(accelerometerBuf_t *accelerometerFIFO, accelerometerBuf_t *accelerometerMax)
{
	uint32_t iFIFO = 0;	//accelerometerFIFO buffer index
	uint32_t iMax = 0;	//accelerometerMax buffer index
	uint16_t pushMax;	//used to push values into the sorted list of MAX vals
	uint16_t popMax;	//used to pop values from the sorted list of MAX vals
	uint16_t newMax;	//a flag to indicate that new MAX has been found

	for (iFIFO = 0; iFIFO < accelerometerFIFO->numVals; iFIFO++)
	{
		//Test current 12-bit val in accelerometerFIFO against all vals in accelerometerMax
		//If new Max is found, pop old Max val into popMax and put new max val into accelerometerMax
		//loop through the remaining positions in accelerometerMax pushing all remaining MAX values down
		//last pushMax val is discarded.
		newMax = 0;
		for (iMax = 0, pushMax = 0; (iMax < 32) && (iMax <= (accelerometerMax->numVals)); iMax++)
		{
			if (newMax)															//push all remaining MAX values down
			{
				popMax = accelerometerMax->val[iMax];							//pop current MAX value
				accelerometerMax->val[iMax] = pushMax;
				pushMax = popMax;
				continue;
			}
			if (accelerometerFIFO->val[iFIFO] >= accelerometerMax->val[iMax])	//found occurance of a new MAX value
			{
				popMax = accelerometerMax->val[iMax];							//pop current MAX value
				accelerometerMax->val[iMax] = accelerometerFIFO->val[iFIFO];
				pushMax = popMax;												
				newMax = 1;
				if(accelerometerMax->numVals<32)
					accelerometerMax->numVals++;
			}
		}
	}
}

int main(int argc, char *argv[])
{
	int32_t rtnVal;
	uint8_t packedFIFO[48];
	uint32_t numBytes;
	char inputFileName[128];
	char outputFileName[128];
	//create local copies of required structs
	accelerometerBuf_t accelerometerFIFO = { 0 };	//most recent read from file or device
	accelerometerBuf_t accelerometerBuf = { 0 };	//lasted 32 12-bit accelerometer values
	accelerometerBuf_t accelerometerMax = { 0 };	//largest 32 12-bit accelerometer values
	
	if (argc == 3)
	{
		sprintf(inputFileName, argv[1]);
		sprintf(outputFileName, argv[2]);
	}
	else
	{
		printf("usage:\nc:>Fitbit_Code_Challenge input_binary_file_name output_value_file_name\n");
	}

	do{
		rtnVal = getAccelerometerFIFOFromFile(inputFileName, (uint8_t*)&packedFIFO, &numBytes);
		unpackAccelerometerFIFO(packedFIFO, numBytes, &accelerometerFIFO);
		processAccelerometerBuf(&accelerometerFIFO, &accelerometerBuf);
		processAccelerometerMax(&accelerometerFIFO, &accelerometerMax);
	}while (rtnVal == 0);
	outputData(outputFileName, &accelerometerBuf, &accelerometerMax);

	return 0;
}

/*
Problem statement:
Write an ANSI C program which takes as arguments the name of a binary input file and a text output file.  The binary input file will contain 12 bit unsigned values.  The output file should contain the 32 largest values from the input file.  It should also contain the last 32 values read from the input file.  Try to write the portions of the code that are not related to IO if you would port it to a memory and speed limited device.  Attached you will find three sample input (.bin) and output (.out) files which you can use for testing.  

Notes:

1. If there are an odd number of 12bit readings in the file, then the last nibble in the file will be zero and can be ignored.

2. The file format should be as follows
   - Start with "--Sorted Max 32 Values--"
   - The 32 largest values in the file, one value per line.  Duplicates are allowed if a large value appears multiple times.  The list should be sorted smallest to largest.
   - Output "--Last 32 Values--" on its own line
   - The last 32 values read, one per line.    They should appear in the order they were read.  The last value read from the file will appear last.

3. If there are fewer than 32 values in the file then whatever is read should be output

4. Your output files should match the output files provided with the given input. 

5. If your program is passed bogus values, it should not core dump.

6. test#.bin is the binary file that corresponds with test#.out   Three test inputs and outputs have been provided.

7. Bonus point for providing a make file or some other build script.

8. Please provide comments in the top of your code describing design decisions / tradeoffs and  if you did any optimizations for speed, memory, etc.  Most of our projects are on MCUs with limited memory powered by a small battery.  Optimizations are not required but a good way to show off IF they work :)
*/