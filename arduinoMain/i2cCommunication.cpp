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

#include "i2cCommunication.h"

#ifdef RASPBERRY_PI

Raspi_i2c Wire;

void dumpVectorBytes(std::vector<byte> in)
{
	std::cout << "Vector byte dump: ";
	std::cout.width(4);
	for (int i = 0; i < in.size(); i++)
	{
		std::cout << (int) in.at(i) << " ";
	}
	std::cout << " Size: " << in.size() << std::endl;
}

//int Raspi_i2c::begin(byte address)
//{
//	_slaveDevice = open("/dev/i2c-slave", O_RDWR);
//	if (_slaveDevice < 0)
//		return I2C_ERR_COULD_NOT_OPEN_SLAVE_DEVICE;
//	if (ioctl(_slaveDevice, I2C_SLAVE, address) < 0)
//		return I2C_ERR_FAILED_SETTING_SLAVE_ADDRESS;
//	return 0;
//}

int Raspi_i2c::beginTransmission(byte address)
{
	_masterDevice = open("/dev/i2c-1", O_RDWR);
	if (_masterDevice < 0)
		return I2C_ERR_COULD_NOT_OPEN_MASTER_DEVICE;
	if (ioctl(_masterDevice, I2C_SLAVE, address) < 0)
		return I2C_ERR_FAILED_SETTING_SLAVE_ADDRESS;
	return 0;
}

int Raspi_i2c::endTransmission(bool in)
{
	int returnVal = 0;
	int writeLen = ::write(_masterDevice, _sendBuffer.data(), _sendBuffer.size());
	//dumpVectorBytes(_sendBuffer);
	if (writeLen != _sendBuffer.size()) returnVal = 4;
	_sendBuffer.erase(_sendBuffer.begin(), _sendBuffer.end());
	close(_masterDevice);
	return returnVal;
}

int Raspi_i2c::write(byte* data, int length)
{
	for (int i = 0; i < length; i++)
	{
		_sendBuffer.push_back(data[i]);
	}
	return length;
}

int Raspi_i2c::available()
{
	const int TEMP_BUFF_SIZE = 32;
	byte tempBuffer[TEMP_BUFF_SIZE];
	int bytesAvailable = ::read(_slaveDevice, tempBuffer, TEMP_BUFF_SIZE);

	if (bytesAvailable >= 0)
	{
		for (int i = 0; i < bytesAvailable; i++)
		{
			_receiveBuffer.push_back(tempBuffer[i]);
		}
		if (bytesAvailable > 0) dumpVectorBytes(_receiveBuffer);
		return _receiveBuffer.size();
	}
	else
	{
		return bytesAvailable;
	}
}

byte Raspi_i2c::read()
{
	if (available() > 0)
	{
		byte returnVal = _receiveBuffer.at(0);
		_receiveBuffer.erase(_receiveBuffer.begin());
		return returnVal;
	}
	else
	{
		return -1;
	}
}
#endif

/* template<class T>
 * T I2C_receiveAnyType(int& errorCode)
 * PURPOSE: Receive any built-in type from I2C.
 * INPUT: 
 *     int& errorCode : I2C error code.
 * OUTPUT:
 *     (T) value of received type.
 */
template<class T>
T I2C_receiveAnyType(int& errorCode)
{
	const size_t typeSize = sizeof(T);
	union ReceiveType // Enables us to cast byte array into built-in type.
	{
		byte bytes[typeSize];
		T value;
	} receiveType;

	int bytesAvailable = 0;
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	bytesAvailable = Wire.available();
#endif

	if (bytesAvailable >= typeSize)
	{
		// Loop over available bytes in I2C buffer, until enough bytes have been read.
		for (int i = 0; i < typeSize; i++)
		{
#if defined(ARDUINO) || defined(RASPBERRY_PI)
			receiveType.bytes[i] = Wire.read();
#endif
		}
		errorCode = 0;
		return receiveType.value; // Convert byte array to type.
	}
	else
	{
		errorCode = I2C_ERR_NOT_ENOUGH_BYTES_IN_INPUT;
		return (T) -1;
	}
}

/* template<class T>
 * void I2C_sendAnyType(T sendValue, int& errorCode)
 * PURPOSE: Send any built-in type over I2C.
 * INPUT: 
 *     T sendValue    : value to be sent.
 *     int& errorCode : I2C error code.
 */
template<class T>
void I2C_sendAnyType(T sendValue, int& errorCode)
{
	const size_t typeSize = sizeof(sendValue);
	byte data[typeSize];
	memmove(&data, &sendValue, typeSize); // Convert input value to byte array for sending.

	int writtenBytes = 0;
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	writtenBytes = Wire.write(data, typeSize);
	//Serial.print("Sent bytes: ");
	//Serial.println(writtenBytes);
#endif

	if (writtenBytes != typeSize)
	{
		errorCode = I2C_ERR_WROTE_TOO_FEW_BYTES;
	}
	else
	{
		errorCode = 0;
	}
}

/* int I2C_requestRandomInt(byte device, int& errorCode)
 * PURPOSE: Request random int from another I2C device.
 * INPUT: 
 *     byte device    : Receiver I2C address.
 *     int& errorCode : I2C error code.
 * OUTPUT:
 *     (int) random value.
 */
int I2C_requestRandomInt(byte device, int& errorCode)
{
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	Wire.beginTransmission(device);
	I2C_sendAnyType(I2C_ADDRESS, errorCode); // Transfer sender address. Necessary, since the line won't remain open.
	I2C_sendAnyType(I2C_REQUEST_RANDOM_INT, errorCode);// Transfer request ID.
	I2C_sendAnyType(I2C_END_OF_MESSAGE, errorCode);// End message.
	errorCode = Wire.endTransmission(true);// Send the bytes, but don't close the connection.
	//Serial.println(errorCode);
#endif
	if (errorCode != 0)
		return 0;

	bool timeoutFlag = false;
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	//sleep(1);
	//waitUntil(Wire.available() >= sizeof(int), 2000, timeoutFlag)
	I2C_respondToRequests();
	//waitUntil(Wire.available() >= sizeof(int), 2000, timeoutFlag)
	//sleep(1);
#endif

	int randomVal;
	if (!timeoutFlag)
	{
		randomVal = I2C_receiveAnyType<int>(errorCode);
	}
	else
	{
		randomVal = 0;
		errorCode = I2C_ERR_TIMEOUT;
	}
	//Wire.endTransmission(true);
	//Serial.println(Wire.available());
	return randomVal;
}

/* float I2C_requestRandomFloat(byte device, int& errorCode)
 * PURPOSE: Request random float from another I2C device.
 * INPUT: 
 *     byte device    : Receiver I2C address.
 *     int& errorCode : I2C error code.
 * OUTPUT:
 *     (float) random value.
 */
float I2C_requestRandomFloat(byte device, int& errorCode)
{
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	Wire.beginTransmission(device);
	I2C_sendAnyType(I2C_ADDRESS, errorCode); // Transfer sender address. Necessary, since the line won't remain open.
	I2C_sendAnyType(I2C_REQUEST_RANDOM_FLOAT, errorCode);// Transfer request ID.
	I2C_sendAnyType(I2C_END_OF_MESSAGE, errorCode);// End message.
	errorCode = Wire.endTransmission(true);// Send the bytes, but don't close the connection.
	//Serial.println(errorCode);
#endif
	if (errorCode != 0)
		return 0;

	bool timeoutFlag = false;
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	waitUntil(Wire.available() >= sizeof(float), 2000, timeoutFlag)
#endif

	float randomVal;
	if (!timeoutFlag)
	{
		randomVal = I2C_receiveAnyType<float>(errorCode);
	}
	else
	{
		randomVal = 0;
		errorCode = I2C_ERR_TIMEOUT;
	}
	//Wire.endTransmission(true);
	//Serial.println(Wire.available());
	return randomVal;
}

/* void I2C_respondToRequests()
 * PURPOSE: Respond to request made by other I2C devices.
 */
void I2C_respondToRequests()
{
#if defined(ARDUINO) || defined(RASPBERRY_PI)
	int avail = Wire.available();
	avail = Wire.available();
	while (Wire.available() >= I2C_MIN_MSG_SIZE) // Loop over requests.
#endif
	{
		// TODO: Add more error recognition.
#if defined(ARDUINO)
		Serial.println(Wire.available());
#endif

		int errorCode;
		// Read first byte, which contains the address of the sender.
		byte requestingAddress = I2C_receiveAnyType<byte>(errorCode);
		// Second byte contains the request ID.
		int request = I2C_receiveAnyType<int>(errorCode);

#if defined(ARDUINO) || defined(RASPBERRY_PI)
		Wire.beginTransmission(requestingAddress);
#endif

		// Recognize the request.
		switch (request)
		{
		case I2C_REQUEST_RANDOM_INT:
		{
			int randomInt = 0;
#if defined(ARDUINO)
			randomInt = (int) random(0, 99);
			Serial.println("Random Int called");
#elif defined(RASPBERRY_PI)
			randomInt = rand() % 100;
			std::cout << "Random Int called, sent: " << randomInt << std::endl;
#endif
			I2C_sendAnyType(randomInt, errorCode);
		}
			break;

		case I2C_REQUEST_RANDOM_FLOAT:
		{
			float randomFloat = 0;
#if defined(ARDUINO)
			randomFloat = random(1, 1000) / 1000.0;
#endif
			I2C_sendAnyType(randomFloat, errorCode);
		}
			break;

		default:
			errorCode = I2C_ERR_UNRECOGNIZED_REQUEST;
			break;
		}

#ifdef ARDUINO
		Serial.println(requestingAddress);
		Serial.println(errorCode);
		Serial.println();
#endif
#if defined(ARDUINO) || defined(RASPBERRY_PI)
		Wire.endTransmission(); // Close connection.
		if (I2C_receiveAnyType<int> (errorCode) != I2C_END_OF_MESSAGE && errorCode == 0)
		{
			errorCode = I2C_ERR_UNKNOWN_RUBBISH_IN_END_INT;
		}
#endif
	}
}

void I2C_OnReceiveDoNothingImmediately(int numBytes)
{
	return;
}

void I2C_transferToRaspi()
{
  #ifdef RASPBERRY_PI_SLAVE
  i2c_int_type sendVal;
  
  if (Wire.available() > 0)
  {
    sendVal = (i2c_int_type) Wire.read();
  }
  else
  {
    sendVal = I2C_END_OF_MESSAGE;
  }

  int errorCode;
  I2C_sendAnyType(sendVal, errorCode);
  #endif
}

