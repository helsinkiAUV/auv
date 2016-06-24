#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "i2cCommunication.h"
#include "Wire.h"
#include "utility.h"
//

#include <Wire.h>

// Sets up the environment for the main loop.
void setup()
{
  Serial.begin(19200);
  Wire.begin(I2C_ADDRESS);
  // Let's us check incoming I2C messages when we deem fit.
  // However, some function must be registered by onReceive(),
  // because without it the receive buffer won't be filled.
  Wire.onReceive(I2C_OnReceiveDoNothingImmediately);
  #ifdef RASPBERRY_PI_SLAVE
  Wire.onRequest(I2C_transferToRaspi);
  #endif
}

void loop()
{
  #ifdef ARDUINO_LEFT

  int errorCode;
  int randomInt = I2C_requestRandomInt(11, errorCode);
  Serial.println(randomInt);

  #elif defined(ARDUINO_RIGHT)

  I2C_respondToRequests();

  #endif

  delay(100);
}

