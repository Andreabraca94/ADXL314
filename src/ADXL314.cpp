/******************************************************************************
  ADXL314.cpp
  Andrea Bracali
  Original Creation Date: October 27, 2024
  https://github.com/Andreabraca94/ADXL314

  Some of this code was copied/tweaked from an Arduino Library for the ADXL314
  Pete Lewis @ SparkFun Electronics
  Original Creation Date: September 19, 2020
  https://github.com/sparkfun/SparkFun_ADXL314_Arduino_Library

  Development environment specifics:

	IDE: Arduino 1.8.13
	
******************************************************************************/

#include "Arduino.h"
#include "ADXL314.h"
#include <Wire.h>
#include <SPI.h>

#define ADXL314_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis

ADXL314::ADXL314() {
	status = ADXL314_OK;
	error_code = ADXL314_NO_ERROR;
	gains[0] = 0.00376390;		// Original gain 0.00376390
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
}

//Initializes the sensor with basic settings
//Returns false if sensor is not detected
boolean ADXL314::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  	_deviceAddress = deviceAddress;
  	_i2cPort = &wirePort;
  	I2C = true;
  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 
	if (checkPartId() == false) // Check for sensor Part ID
		return (false);
  	return (true); //We're all setup!
}

//Returns true if I2C device ack's
boolean ADXL314::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

//Initializes the sensor with basic settings via SPI
//Returns false if sensor is not detected
boolean ADXL314::beginSPI(uint8_t CS_pin, SPIClass &spiPort, uint32_t spi_freq)
{
	_CS = CS_pin;
	_spiPort = &spiPort;
	I2C = false;
	SPIfreq = spi_freq;
	_spiPort->begin();
	_spiPort->beginTransaction(SPISettings(SPIfreq, MSBFIRST, SPI_MODE3));
	pinMode(_CS, OUTPUT);
	digitalWrite(_CS, HIGH);

	// since we can't simply check for an "ACK" like in I2C,
	// we will check PART ID, to verify that it's there and working
  	if (checkPartId() == false) // Check for sensor Part ID
    	return (false);

  	return (true); //We're all setup!
}

// Returns true if device's part ID register is correct
boolean ADXL314::checkPartId() {
	byte _b;
	readFrom(ADXL314_DEVID_0, 1, &_b);
	if(_b == ADXL314_DEVID_0_RSP_EXPECTED)
		return (true);
	return (false);
}

boolean ADXL314::dataReady() {
	return getRegisterBit(ADXL314_INT_SOURCE, ADXL314_INT_DATA_READY_BIT);	// check the dataReady bit 
}

boolean ADXL314::updateIntSourceStatuses() {
	byte _b;
	readFrom(ADXL314_INT_SOURCE, 1, &_b);
	intSource.dataReady = ((_b >> ADXL314_INT_DATA_READY_BIT) & 1);
	intSource.activity = ((_b >> ADXL314_INT_ACTIVITY_BIT) & 1);
	intSource.inactivity = ((_b >> ADXL314_INT_INACTIVITY_BIT) & 1);
	intSource.watermark = ((_b >> ADXL314_INT_WATERMARK_BIT) & 1);
	intSource.overrun = ((_b >> ADXL314_INT_OVERRUN_BIT) & 1);

	return (true);
}

boolean ADXL314::standby() {
	// clears the measure bit, putting decive in standby mode, ready for configuration
	setRegisterBit(ADXL314_POWER_CTL, ADXL314_MEASURE_BIT, false);
	return (true);
}

boolean ADXL314::measureModeOn() {
	// sets the measure bit, putting decive in measure mode, ready for reading data
	setRegisterBit(ADXL314_POWER_CTL, ADXL314_MEASURE_BIT, true);
	return (true);
}


/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Class Variables:  x, y and z          */


void ADXL314::readAccel() {
	readFrom(ADXL314_DATA_X0, ADXL314_TO_READ, _buff);	// Read Accel Data from ADXL345

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
	y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
	z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
}

/*************************** AUTOSLEEP BIT **************************/
/*                            ~ ON & OFF                           	*/
boolean ADXL314::autosleepOn() {
	// sets the autosleep bit
	// note, prior to calling this, 
	// you will need to set THRESH_INACT and TIME_INACT.
	// set the link bit, to "link" activity and inactivity sensing
	setRegisterBit(ADXL314_POWER_CTL, ADXL314_LINK_BIT, true);
	
	// set the autosleep
	setRegisterBit(ADXL314_POWER_CTL, ADXL314_AUTOSLEEP_BIT, true);
	return (true);
}

boolean ADXL314::autosleepOff() {
	// clears the autosleep bit
	setRegisterBit(ADXL314_POWER_CTL, ADXL314_AUTOSLEEP_BIT, false);
	return (true);
}

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
boolean ADXL314::getSelfTestBit() {
	return getRegisterBit(ADXL314_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
void ADXL314::setSelfTestBit(boolean selfTestBit) {
	setRegisterBit(ADXL314_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
boolean ADXL314::getSpiBit() {
	return getRegisterBit(ADXL314_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
void ADXL314::setSpiBit(boolean spiBit) {
	setRegisterBit(ADXL314_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
boolean ADXL314::getInterruptLevelBit() {
	return getRegisterBit(ADXL314_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
void ADXL314::setInterruptLevelBit(boolean interruptLevelBit) {
	setRegisterBit(ADXL314_DATA_FORMAT, 5, interruptLevelBit);
}


/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
boolean ADXL314::getJustifyBit() {
	return getRegisterBit(ADXL314_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
void ADXL314::setJustifyBit(boolean justifyBit) {
	setRegisterBit(ADXL314_DATA_FORMAT, 2, justifyBit);
}


/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
void ADXL314::setAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}
void ADXL314::getAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 3.9 mg/LSB
void ADXL314::setAxisOffset(int x, int y, int z) {
	writeTo(ADXL314_OFSX, byte (x));
	writeTo(ADXL314_OFSY, byte (y));
	writeTo(ADXL314_OFSZ, byte (z));
}

void ADXL314::getAxisOffset(int* x, int* y, int*z) {
	byte _b;
	readFrom(ADXL314_OFSX, 1, &_b);
	*x = int (_b);
	readFrom(ADXL314_OFSY, 1, &_b);
	*y = int (_b);
	readFrom(ADXL314_OFSZ, 1, &_b);
	*z = int (_b);
}



/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared
// with the Value in the THRESH_ACT Register.
// The Scale Factor is 784mg/LSB.
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void ADXL314::setActivityThreshold(int activityThreshold) {
	activityThreshold = constrain(activityThreshold,0,255);
	byte _b = byte (activityThreshold);
	writeTo(ADXL314_THRESH_ACT, _b);
}

// Gets the THRESH_ACT byte
int ADXL314::getActivityThreshold() {
	byte _b;
	readFrom(ADXL314_THRESH_ACT, 1, &_b);
	return int (_b);
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is
//  Compared with the value in the THRESH_INACT Register.
// Scale Factor is 784mg/LSB.
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void ADXL314::setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	byte _b = byte (inactivityThreshold);
	writeTo(ADXL314_THRESH_INACT, _b);
}

int ADXL314::getInactivityThreshold() {
	byte _b;
	readFrom(ADXL314_THRESH_INACT, 1, &_b);
	return int (_b);
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared.
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB.
// Value Must Be Between 0 and 255.
void ADXL314::setTimeInactivity(int timeInactivity) {
	timeInactivity = constrain(timeInactivity,0,255);
	byte _b = byte (timeInactivity);
	writeTo(ADXL314_TIME_INACT, _b);
}

int ADXL314::getTimeInactivity() {
	byte _b;
	readFrom(ADXL314_TIME_INACT, 1, &_b);
	return int (_b);
}



/************************** ACTIVITY BITS ***************************/
/*                                                                  */
boolean ADXL314::isActivityXEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 6);
}
boolean ADXL314::isActivityYEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 5);
}
boolean ADXL314::isActivityZEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 4);
}
boolean ADXL314::isInactivityXEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 2);
}
boolean ADXL314::isInactivityYEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 1);
}
boolean ADXL314::isInactivityZEnabled() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 0);
}

void ADXL314::setActivityX(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 6, state);
}
void ADXL314::setActivityY(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 5, state);
}
void ADXL314::setActivityZ(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 4, state);
}
void ADXL314::setActivityXYZ(boolean stateX, boolean stateY, boolean stateZ) {
	setActivityX(stateX);
	setActivityY(stateY);
	setActivityZ(stateZ);
}
void ADXL314::setInactivityX(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 2, state);
}
void ADXL314::setInactivityY(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 1, state);
}
void ADXL314::setInactivityZ(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 0, state);
}
void ADXL314::setInactivityXYZ(boolean stateX, boolean stateY, boolean stateZ) {
	setInactivityX(stateX);
	setInactivityY(stateY);
	setInactivityZ(stateZ);
}

boolean ADXL314::isActivityAc() {
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 7);
}
boolean ADXL314::isInactivityAc(){
	return getRegisterBit(ADXL314_ACT_INACT_CTL, 3);
}

void ADXL314::setActivityAc(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 7, state);
}
void ADXL314::setInactivityAc(boolean state) {
	setRegisterBit(ADXL314_ACT_INACT_CTL, 3, state);
}


/************************** LOW POWER BIT ***************************/
/*                                                                  */
boolean ADXL314::isLowPower(){
	return getRegisterBit(ADXL314_BW_RATE, 4);
}
void ADXL314::lowPowerOn() {
	setRegisterBit(ADXL314_BW_RATE, 4, true);
}
void ADXL314::lowPowerOff() {
	setRegisterBit(ADXL314_BW_RATE, 4, false);
}

/*************************** RATE BITS ******************************/
/*                                                                  */
double ADXL314::getRate(){
	byte _b;
	readFrom(ADXL314_BW_RATE, 1, &_b);
	_b &= 0b00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL314::setRate(double rate){
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) {
		readFrom(ADXL314_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & 0b11110000);
		writeTo(ADXL314_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
void ADXL314::setBandwidth(byte bw){
 	writeTo(ADXL314_BW_RATE, bw);
}

byte ADXL314::getBandwidth(){
	byte _b;
	readFrom(ADXL314_BW_RATE, 1, &_b);
	return _b;
}




/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL314_DATA_READY);
boolean ADXL314::triggered(byte interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

byte ADXL314::getInterruptSource() {
	byte _b;
	readFrom(ADXL314_INT_SOURCE, 1, &_b);
	return _b;
}

boolean ADXL314::getInterruptSource(byte interruptBit) {
	return getRegisterBit(ADXL314_INT_SOURCE,interruptBit);
}

boolean ADXL314::getInterruptMapping(byte interruptBit) {
	return getRegisterBit(ADXL314_INT_MAP,interruptBit);
}

// /*********************** INTERRUPT MAPPING **************************/
// /*         Set the Mapping of an Interrupt to pin1 or pin2          */
// // eg: setInterruptMapping(ADXL314_INT_WATERMARK_BIT,ADXL314_INT2_PIN);
 void ADXL314::setInterruptMapping(byte interruptBit, boolean interruptPin) {
 	setRegisterBit(ADXL314_INT_MAP, interruptBit, interruptPin);
 }

boolean ADXL314::isInterruptEnabled(byte interruptBit) {
	return getRegisterBit(ADXL314_INT_ENABLE,interruptBit);
}

void ADXL314::setInterrupt(byte interruptBit, boolean state) {
	setRegisterBit(ADXL314_INT_ENABLE, interruptBit, state);
}

void ADXL314::ActivityINT(boolean status) {
	if(status) {
		setInterrupt( ADXL314_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( ADXL314_INT_ACTIVITY_BIT,   0);
	}
}
void ADXL314::InactivityINT(boolean status) {
	if(status) {
		setInterrupt( ADXL314_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( ADXL314_INT_INACTIVITY_BIT, 0);
	}
}

void ADXL314::DataReadyINT(boolean status) {
	if(status) {
		setInterrupt( ADXL314_INT_DATA_READY_BIT, 1);
	}
	else {
		setInterrupt( ADXL314_INT_DATA_READY_BIT, 0);
	}
}

void ADXL314::WatermarkINT(boolean status) {
	if(status) {
		setInterrupt( ADXL314_INT_WATERMARK_BIT, 1);
	}
	else {
		setInterrupt( ADXL314_INT_WATERMARK_BIT, 0);
	}
}

void ADXL314::OverrunINT(boolean status) {
	if(status) {
		setInterrupt( ADXL314_INT_OVERRUN_BIT, 1);
	}
	else {
		setInterrupt( ADXL314_INT_OVERRUN_BIT, 0);
	}
}


/*************************** FIFO MODE SETTING **************************/
/*          	                GET & SET                         		*/
byte ADXL314::getFifoMode() {
	byte _b;
	readFrom(ADXL314_FIFO_CTL, 1, &_b);
	byte mode = (_b & 0b11000000);
	mode = (mode >> 6);
	return mode;
}

void ADXL314::setFifoMode(byte mode) {
	byte _s = (mode << 6);
	byte _b;
	readFrom(ADXL314_FIFO_CTL, 1, &_b);
	_s |= (_b & 0b00111111);
	writeTo(ADXL314_FIFO_CTL, _s);
}

byte ADXL314::getFifoSamplesThreshhold() {
	byte _b;
	readFrom(ADXL314_FIFO_CTL, 1, &_b);
	byte samples = (_b & 0b00011111);
	return samples;
}

void ADXL314::setFifoSamplesThreshhold(byte samples) {
	byte _s = samples;
	byte _b;
	readFrom(ADXL314_FIFO_CTL, 1, &_b);
	_s |= (_b & 0b11100000);
	writeTo(ADXL314_FIFO_CTL, _s);
}

byte ADXL314::getFifoEntriesAmount() {
	byte _b;
	readFrom(ADXL314_FIFO_STATUS, 1, &_b);
	byte entries = (_b & 0b00111111);
	return entries;
}

void ADXL314::clearFifo() {
	byte mode = getFifoMode(); // get current mode
	setFifoMode(ADXL314_FIFO_MODE_BYPASS); // set mode to bypass temporarily to clear contents
	setFifoMode(mode); // return mode to previous selection.

}

void ADXL314::setRegisterBit(byte regAdress, int bitPos, boolean state) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);
}

boolean ADXL314::getRegisterBit(byte regAdress, int bitPos) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

/********************************************************************/
/*                                                                  */
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
void ADXL314::printAllRegister() {
	byte _b;
	Serial.print("0x00: ");
	readFrom(0x00, 1, &_b);
	print_byte(_b);
	Serial.println("");
	int i;
	for (i=29;i<=57;i++){
		Serial.print("0x");
		Serial.print(i, HEX);
		Serial.print(": ");
		readFrom(i, 1, &_b);
		print_byte(_b);
		Serial.println("");
	}
}

void print_byte(byte val){
	int i;
	Serial.print("B");
	for(i=7; i>=0; i--){
		Serial.print(val >> i & 1, BIN);
	}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void ADXL314::writeTo(byte address, byte val) {
	if(I2C) {
		writeToI2C(address, val);
	}
	else {
		writeToSPI(address, val);
	}
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
void ADXL314::readFrom(byte address, uint8_t num, byte _buff[]) {
	if(I2C) {
		readFromI2C(address, num, _buff);	// If I2C Communication
	}
	else {
		readFromSPI(address, num, _buff);	// If SPI Communication
	}
}

/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
void ADXL314::writeToI2C(byte _address, byte _val) {
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(_address);
	_i2cPort->write(_val);
	_i2cPort->endTransmission();
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
void ADXL314::readFromI2C(byte address, uint8_t num, byte _buff[]) {
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(address);
	_i2cPort->endTransmission();

//	_i2cPort->beginTransmission(ADXL314_DEVICE);
// _i2cPort->reqeustFrom contains the beginTransmission and endTransmission in it. 
	_i2cPort->requestFrom(_deviceAddress, num);  // Request 6 Bytes

	int i = 0;
	while(_i2cPort->available())
	{
		_buff[i] = _i2cPort->read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = ADXL314_ERROR;
		error_code = ADXL314_READ_ERROR;
	}
//	_i2cPort->endTransmission();
}

/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
void ADXL314::writeToSPI(byte __reg_address, byte __val) {
  _spiPort->beginTransaction(SPISettings(SPIfreq, MSBFIRST, SPI_MODE3));
  digitalWrite(_CS, LOW);
  _spiPort->transfer(__reg_address);
  _spiPort->transfer(__val);
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();
}

/*************************** READ FROM SPI **************************/
/*                                                                  */
void ADXL314::readFromSPI(byte __reg_address, int num, byte _buff[]) {
  // Read: Most Sig Bit of Reg Address Set
  char _address = 0x80 | __reg_address;
  // If Multi-Byte Read: Bit 6 Set
  if(num > 1) {
  	_address = _address | 0x40;
  }


  _spiPort->beginTransaction(SPISettings(SPIfreq, MSBFIRST, SPI_MODE3));
  digitalWrite(_CS, LOW);
  _spiPort->transfer(_address);		// Transfer Starting Reg Address To Be Read
  for(int i=0; i<num; i++){
    _buff[i] = _spiPort->transfer(0x00);
  }
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();
}
