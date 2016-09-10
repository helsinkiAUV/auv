
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "OrientationSensor.h"

void setup() 
{
  Serial.begin(19200);
}

void loop() 
{
  int errorFlag;
  OrientationSensor imu(errorFlag);
  
  Serial.print("Current Temperature: ");
  Serial.print(imu.temperature());
  Serial.println(" C");
  //Serial.println("");



  Serial.print("Current magnetic heading: ");
  Serial.print(imu.heading()); 
  Serial.println("°");
  Serial.println("");

  imu.displayCalStatus();
  
  delay(1000);
}
