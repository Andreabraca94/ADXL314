/******************************************************************************
  ADXL314_Registers.h
  ADXL314 Register Map
  Andrea Bracali
  Original Creation Date: October 27, 2024
  https://github.com/Andreabraca94/ADXL314

  Some of this code was copied/tweaked from an Arduino Library for the ADXL314
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL314_Arduino_Library

  This file defines all registers internal to the ADXL314.

  Development environment specifics:

	IDE: Arduino 1.8.13

******************************************************************************/

#ifndef __ADXL314_Registers_H__
#define __ADXL314_Registers_H__

/////////////////////////////////////////
// ADXL314 Registers //
/////////////////////////////////////////
#define ADXL314_DEVID_0				0x00
#define ADXL314_OFSX				0x1E
#define ADXL314_OFSY				0x1F
#define ADXL314_OFSZ				0x20
#define ADXL314_THRESH_ACT			0x24
#define ADXL314_THRESH_INACT			0x25
#define ADXL314_TIME_INACT			0x26
#define ADXL314_ACT_INACT_CTL			0x27
#define ADXL314_BW_RATE				0x2C
#define ADXL314_POWER_CTL			0x2D
#define ADXL314_INT_ENABLE			0x2E
#define ADXL314_INT_MAP				0x2F
#define ADXL314_INT_SOURCE			0x30
#define ADXL314_DATA_FORMAT			0x31
#define ADXL314_DATA_X0				0x32
#define ADXL314_DATA_X1				0x33
#define ADXL314_DATA_Y0				0x34
#define ADXL314_DATA_Y1				0x35
#define ADXL314_DATA_Z0				0x36
#define ADXL314_DATA_Z1				0x37
#define ADXL314_FIFO_CTL			0x38
#define ADXL314_FIFO_STATUS			0x39

////////////////////////////////
// ADXL314 Responses //
////////////////////////////////
#define ADXL314_DEVID_0_RSP_EXPECTED			0xE5

#endif
