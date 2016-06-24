/*
 * I2C communication functions.
 * Created by Juho Iipponen on May 26th, 2016.
 *
 * This file is part of the University of Helsinki AUV source code.
 *
 * The AUV source code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * The AUV source code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the source code.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#ifndef I2C_COMMUNICATION_H_
#define I2C_COMMUNICATION_H_

#include "auv.h"

#ifdef ARDUINO
#include "Arduino.h"
#include "../../libraries/Wire/Wire.h"
#endif

#ifdef RASPBERRY_PI
#include<stdio.h>
#include<unistd.h>
#include<fcntl.h>
#include<curses.h>
#include <vector>
#include <string>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
//#include "bsc-slave.h"
#include <stdlib.h>
//#include "rPodI2C.h"
#endif

#ifndef ARDUINO
  #include <string.h>
#endif
#include "utility.h"

// Typedefs
#ifdef ARDUINO
typedef long i2c_int_type;
#elif defined(RASPBERRY_PI)
typedef unsigned char byte;
typedef int i2c_int_type;
#endif

// Addresses.
#ifdef ARDUINO_LEFT
const byte I2C_ADDRESS = 10;
#elif defined(ARDUINO_RIGHT)
const byte I2C_ADDRESS = 11;
#elif defined(RASPBERRY_PI_SLAVE) || defined(RASPBERRY_PI)
const byte I2C_ADDRESS = 12;
#endif

const int I2C_MIN_MSG_SIZE = sizeof(byte) + 2*sizeof(i2c_int_type);

const i2c_int_type I2C_END_OF_MESSAGE = -2147483648;
const i2c_int_type I2C_NO_BYTES_IN_RX_BUFFER = 2147483647;

// I2C requests
const i2c_int_type I2C_REQUEST_RANDOM_INT = 1;
const i2c_int_type I2C_REQUEST_RANDOM_FLOAT = 2;

// Error status messages
const int I2C_ERR_TIMEOUT = 1;
const int I2C_ERR_NOT_ENOUGH_BYTES_IN_INPUT = 2;
const int I2C_ERR_WROTE_TOO_FEW_BYTES = 3;
const int I2C_ERR_UNRECOGNIZED_REQUEST = 4;
const int I2C_ERR_UNKNOWN_RUBBISH_IN_END_INT = 5; // Last two bytes were not I2C_END_OF_MESSAGE. Strange...
const int I2C_ERR_COULD_NOT_OPEN_SLAVE_DEVICE = 6;
const int I2C_ERR_COULD_NOT_OPEN_MASTER_DEVICE = 7;
const int I2C_ERR_FAILED_SETTING_SLAVE_ADDRESS = 8;
const i2c_int_type I2C_ERR_READING_SLAVE = -1000;  


#ifdef RASPBERRY_PI
class Raspi_i2c
{
private:
	std::vector<byte> _receiveBuffer; // Need contigous storage, so e.g. queue won't do!
	std::vector<byte> _sendBuffer;
	int _availableLength;
	int _slaveDevice;
	int _masterDevice;
	i2c_int_type read_i2c_int_type_from_slave_arduino();
public:
	Raspi_i2c() : _availableLength(0), _masterDevice(0) {};
	//int begin(byte address);
	int beginTransmission(byte address);
	int endTransmission(bool in = true);
	int write(byte* data, int length);
	int available();
	byte read();
};
extern Raspi_i2c Wire;
#endif

template<class T>
T I2C_receiveAnyType(int& errorCode);

template<class T>
void I2C_sendAnyType(T sendValue, int& errorCode);

int I2C_requestRandomInt(byte device, int& errorCode);

float I2C_requestRandomFloat(byte device, int& errorCode);

void I2C_respondToRequests();

void I2C_OnReceiveDoNothingImmediately(int numBytes);

void I2C_transferToRaspi();

#endif
