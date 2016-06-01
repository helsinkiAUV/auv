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
  #ifdef ARDUINO
  bytesAvailable = Wire.available();
  #endif
  
  if (bytesAvailable >= typeSize)
  { 
    // Loop over available bytes in I2C buffer, until enough bytes have been read.
    for (int i = 0; i < typeSize; i++)
    {
      #ifdef ARDUINO
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
  #ifdef ARDUINO
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
  #ifdef ARDUINO
  Wire.beginTransmission(device);
  I2C_sendAnyType(I2C_ADDRESS, errorCode); // Transfer sender address. Necessary, since the line won't remain open.
  I2C_sendAnyType(I2C_REQUEST_RANDOM_INT, errorCode); // Transfer request ID.
  I2C_sendAnyType(I2C_END_OF_MESSAGE, errorCode); // End message.
  errorCode = Wire.endTransmission(true); // Send the bytes, but don't close the connection.
  //Serial.println(errorCode);
  #endif
  if (errorCode != 0) return 0;
  
  bool timeoutFlag = false;
  #ifdef ARDUINO
  waitUntil(Wire.available() >= sizeof(int), 2000, timeoutFlag)
  #endif
  
  int randomVal;
  if (!timeoutFlag)
  {
    randomVal = I2C_receiveAnyType<int> (errorCode);
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
  #ifdef ARDUINO
  Wire.beginTransmission(device);
  I2C_sendAnyType(I2C_ADDRESS, errorCode); // Transfer sender address. Necessary, since the line won't remain open.
  I2C_sendAnyType(I2C_REQUEST_RANDOM_FLOAT, errorCode); // Transfer request ID.
  I2C_sendAnyType(I2C_END_OF_MESSAGE, errorCode); // End message.
  errorCode = Wire.endTransmission(true); // Send the bytes, but don't close the connection.
  //Serial.println(errorCode);
  #endif
  if (errorCode != 0) return 0;
  
  bool timeoutFlag = false;
  #ifdef ARDUINO
  waitUntil(Wire.available() >= sizeof(float), 2000, timeoutFlag)
  #endif
  
  float randomVal;
  if (!timeoutFlag)
  {
    randomVal = I2C_receiveAnyType<float> (errorCode);
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
  #ifdef ARDUINO
  while (Wire.available()) // Loop over requests.
  #endif
  {
    // TODO: Add more error recognition.
    #ifdef ARDUINO
    Serial.println(Wire.available());
    #endif
  
    int errorCode;
    // Read first byte, which contains the address of the sender.
    byte requestingAddress = I2C_receiveAnyType<byte> (errorCode);
    // Second byte contains the request ID.
    int request = I2C_receiveAnyType<int> (errorCode);
    
    #ifdef ARDUINO
    Wire.beginTransmission(requestingAddress);
    #endif
  
    // Recognize the request.
    switch (request)
    {
      case I2C_REQUEST_RANDOM_INT:
      {
        int randomInt = 0;
        #ifdef ARDUINO
        randomInt = (int) random(1, 100);
        Serial.println("Random Int called");
        #endif
        I2C_sendAnyType(randomInt, errorCode);
      }
        break;
        
      case I2C_REQUEST_RANDOM_FLOAT:
      {
        float randomFloat = 0;
        #ifdef ARDUINO
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

