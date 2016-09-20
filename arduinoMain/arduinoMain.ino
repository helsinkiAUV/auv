
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "OrientationSensor.h"

  int errorFlag = 0;
OrientationSensor orient;

void setup() 
{
  Serial.begin(19200);
  orient.begin(errorFlag);
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
  //orient.recalibrate(errorFlag);
  Serial.println("Alive");
  Serial.println("Alive");
//  int errorFlag;
//  OrientationSensor imu(errorFlag);
  Serial.println(errorFlag);  
  Serial.print("Current Temperature: ");
  Serial.print(orient.temperature());
  Serial.println(" C");
  //Serial.println("");
//
//
//
  Serial.print("Current magnetic heading: ");
  Serial.print(orient.heading()); 
  Serial.println("°");
  Serial.println("");

  orient.displayCalStatus();
  //orient.displaySensorStatus();

  imu::Vector<3> gravity = orient.gravityField();
  Serial.print("Gravity: ");
  Serial.print(gravity.x());
  Serial.print(" ");
  Serial.print(gravity.y());
  Serial.print(" ");
  Serial.println(gravity.z());
  
  delay(1000);
}
