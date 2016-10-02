
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "ServoAuv.h"
#include "OrientationSensor.h"

int errorFlag = 0;
OrientationSensor orient;

void setup() 
{
  Serial.begin(19200);
  //orient.begin(errorFlag);
//  Serial.println("Alive");
//  while(!orient.isFullyCalibrated())
//  {
//    Serial.println("Alive");
//    orient.displayCalStatus();
//    Serial.println(errorFlag);
//    delay(1000);
//  }
  
}

void loop() 
{
  ServoAuv servo(11);
  for (int i = -6; i <= 6; i++)
  {
    servo.turnTo(i);
    delay(1000);
  }
  
  delay(1000);
}

