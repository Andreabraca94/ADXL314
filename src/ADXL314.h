/******************************************************************************
  ADXL314 Arduino Library - ADXL314 Header File
  Andrea Bracali
  Original Creation Date: October 27, 2024
  https://github.com/Andreabraca94/ADXL314

  Some of this code was copied/tweaked from an Arduino Library for the ADXL345
  Written by Pete Lewis @ SparkFun Electronics
  Created: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL313_Arduino_Library
  
  This file defines all registers internal to the ADXL314.

  Development environment specifics:

	IDE: Arduino 1.8.13

******************************************************************************/

#ifndef __SparkFunADXL314_H__
#define __SparkFunADXL314_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include "ADXL314_Registers.h"
#include "Wire.h"
#include <SPI.h>

#define ADXL314_I2C_ADDRESS_DEFAULT 	0x1D
#define ADXL314_I2C_ADDRESS_ALT 0x53
#define ADXL314_CS_PIN_DEFAULT 10

 /************************** INTERRUPT PINS **************************/ //NEED TO CHECK THESE FOR 314!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ADXL314_INT1_PIN		0x00		//INT1: 0
#define ADXL314_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL314_INT_DATA_READY_BIT		0x07
#define ADXL314_INT_ACTIVITY_BIT		0x04
#define ADXL314_INT_INACTIVITY_BIT		0x03
#define ADXL314_INT_WATERMARK_BIT		0x01
#define ADXL314_INT_OVERRUN_BIT			0x00

#define ADXL314_DATA_READY				0x07
#define ADXL314_ACTIVITY				0x04
#define ADXL314_INACTIVITY				0x03
#define ADXL314_WATERMARK				0x01
#define ADXL314_OVERRUN					0x00


 /********************** POWER_CTL BIT POSITION **********************/
#define ADXL314_I2C_DISABLE_BIT			0x06
#define ADXL314_LINK_BIT			0x05
#define ADXL314_AUTOSLEEP_BIT			0x04
#define ADXL314_MEASURE_BIT			0x03
#define ADXL314_SLEEP_BIT			0x02

 /********************** BANDWIDTH RATE CODES (HZ) *******************/
#define ADXL314_BW_1600			0xF			// 1111		IDD = 170uA
#define ADXL314_BW_800			0xE			// 1110		IDD = 115uA
#define ADXL314_BW_400			0xD			// 1101		IDD = 170uA
#define ADXL314_BW_200			0xC			// 1100		IDD = 170uA (115 low power)
#define ADXL314_BW_100			0xB			// 1011		IDD = 170uA (82 low power)
#define ADXL314_BW_50			0xA			// 1010		IDD = 170uA (64 in low power)
#define ADXL314_BW_25			0x9			// 1001		IDD = 115uA (57 in low power)
#define ADXL314_BW_12_5		    	0x8			// 1000		IDD = 82uA (50 in low power)
#define ADXL314_BW_6_25			0x7			// 0111		IDD = 65uA (43 in low power)
#define ADXL314_BW_3_125		0x6			// 0110		IDD = 57uA

 /********************** FIFO MODE OPTIONS ***************************/
#define ADXL314_FIFO_MODE_BYPASS	0x00
#define ADXL314_FIFO_MODE_FIFO		0x01
#define ADXL314_FIFO_MODE_STREAM	0x02
#define ADXL314_FIFO_MODE_TRIGGER	0x03

 /****************************** ERRORS ******************************/
#define ADXL314_OK		1		// No Error
#define ADXL314_ERROR		0		// Error Exists
#define ADXL314_NO_ERROR	0		// Initial State
#define ADXL314_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL314_BAD_ARG		2		// Bad Argument


//  ADXL314Interrupts
//
//  This is used by the ADXL314 class to hold interrupt settings and statuses from the most recent read of INT_SOURCE. 
//  It is public within that class and the user is expected to write desired values into the settings before calling
//  .setInterrupts();
struct ADXL314IntSource
{
  public:
    boolean dataReady;
    boolean activity;
	boolean inactivity;
	boolean watermark;
	boolean overrun;
};


class ADXL314
{
public:
	
	// We'll store the accelerometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readAdxl() first, before using these variables!
	// These values are the RAW signed 16-bit readings from the sensor.
	int16_t x, y, z; // x, y, and z axis readings of the accelerometer

	boolean status;					// Set When Error Exists 

	byte error_code;				// Initial State
	double gains[3];				// Counts to Gs

	// INT_SOURCE register bit statuses
	// used to allow a single read of the INT_SOURCE register,
	// and then later check the status of each bit (stored individually in class varaibles)
	ADXL314IntSource intSource;


	// ADXL314 class constructor
	// The constructor will set up with default settings via I2C

	ADXL314();
		
	// begin() -- Initialize the accelerometer
	// This will set up the scale and output rate of each sensor. The values set
	// in the IMUSettings struct will take effect after calling this function.
	// INPUTS:
	// - agAddress - Sets either the I2C address of the accel/gyro or SPI chip 
	//   select pin connected to the CS_XG pin.
	// - mAddress - Sets either the I2C address of the magnetometer or SPI chip 
	//   select pin connected to the CS_M pin.
	// - i2C port (Note, only on "begin()" funtion, for use with I2C com interface)
	//   defaults to Wire, but if hardware supports it, can use other TwoWire ports.
	//   **For SPI use "beginSPI()", and only send first two address arguments.
	boolean begin(uint8_t address = ADXL314_I2C_ADDRESS_DEFAULT, TwoWire &wirePort = Wire); //By default use the default I2C addres, and use Wire port
	boolean beginSPI(uint8_t CS_pin = ADXL314_CS_PIN_DEFAULT, SPIClass &spiPort = SPI, uint32_t spi_freq = 5000000); //By default use the default CS pin, and use SPI port


	boolean isConnected();
	boolean checkPartId();

	// dataReady() -- REads the entire INT_Source register, and checks the DATA_READY bit
	// to see if new data is available.
	// **Note, this will also clear any other INT source bits.
	// If you need to know the other int source bits, then use updateIntSourceStatuses()
	// Output:	1 - New data available
	//			0 - No new data available
	boolean dataReady();

	// updateIntSourceStatuses() -- Reads the entire INT_Source register, 
	// and stores all of the int statuses in class variables.
	// note, this will clear all INT source bits.
	// Output:	1 - function completed
	//			0 - Communication failure
	boolean updateIntSourceStatuses();

	boolean measureModeOn();

	boolean standby();
	
	// readAccel() -- Read the sensors output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' x, y, and z variables. Read
	// those _after_ calling readAccel().
	void readAccel();

	boolean autosleepOn();
	boolean autosleepOff();

	byte getFifoMode();
	void setFifoMode(byte mode);
	void setFifoSamplesThreshhold(byte samples);
	byte getFifoSamplesThreshhold();
	byte getFifoEntriesAmount();
	void clearFifo();
	
	void setAxisGains(double *_gains);
	void getAxisGains(double *_gains);
	void setAxisOffset(int x, int y, int z);
	void getAxisOffset(int* x, int* y, int*z);
	void setActivityThreshold(int activityThreshold);
	int getActivityThreshold();
	void setInactivityThreshold(int inactivityThreshold);
	int getInactivityThreshold();
	void setTimeInactivity(int timeInactivity);
	int getTimeInactivity();
	
	boolean isActivityXEnabled();
	boolean isActivityYEnabled();
	boolean isActivityZEnabled();
	boolean isInactivityXEnabled();
	boolean isInactivityYEnabled();
	boolean isInactivityZEnabled();
	boolean isActivityAc();
	boolean isInactivityAc();
	void setActivityAc(boolean state);
	void setInactivityAc(boolean state);
	
	void setActivityX(boolean state);
	void setActivityY(boolean state);
	void setActivityZ(boolean state);
	void setActivityXYZ(boolean stateX, boolean stateY, boolean stateZ);
	void setInactivityX(boolean state);
	void setInactivityY(boolean state);
	void setInactivityZ(boolean state);
	void setInactivityXYZ(boolean stateX, boolean stateY, boolean stateZ);
	
	boolean isActivitySourceOnX();
	boolean isActivitySourceOnY();
	boolean isActivitySourceOnZ();
	boolean isAsleep();
	
	boolean isLowPower();
	void lowPowerOn();
	void lowPowerOff();
	double getRate();
	void setRate(double rate);
	void setBandwidth(byte bw);
	byte getBandwidth();
	
	boolean triggered(byte interrupts, int mask);
	
	byte getInterruptSource();
	boolean getInterruptSource(byte interruptBit);
	boolean getInterruptMapping(byte interruptBit);
	void setInterruptMapping(byte interruptBit, boolean interruptPin);
	boolean isInterruptEnabled(byte interruptBit);
	void setInterrupt(byte interruptBit, boolean state);
	void InactivityINT(boolean status);
	void ActivityINT(boolean status);
	void DataReadyINT(boolean status);
	void WatermarkINT(boolean status);
	void OverrunINT(boolean status);
	
	boolean getSelfTestBit();
	void setSelfTestBit(boolean selfTestBit);
	boolean getSpiBit();
	void setSpiBit(boolean spiBit);
	boolean getInterruptLevelBit();
	void setInterruptLevelBit(boolean interruptLevelBit);
	boolean getJustifyBit();
	void setJustifyBit(boolean justifyBit);
	void printAllRegister();
	
private:

	TwoWire *_i2cPort;
	SPIClass *_spiPort;
	uint8_t _deviceAddress;

	void writeTo(byte address, byte val);
	void writeToI2C(byte address, byte val);
	void writeToSPI(byte address, byte val);
	void readFrom(byte address, uint8_t num, byte buff[]);
	void readFromI2C(byte address, uint8_t num, byte buff[]);
	void readFromSPI(byte address, int num, byte buff[]);
	void setRegisterBit(byte regAdress, int bitPos, boolean state);
	boolean getRegisterBit(byte regAdress, int bitPos);  
	
	byte _buff[6] ;		//	6 Bytes Buffer
	int _CS = ADXL314_CS_PIN_DEFAULT;
	boolean I2C = true;
	unsigned long SPIfreq;
};
void print_byte(byte val);

#endif // SFE_ADXL314_H //
