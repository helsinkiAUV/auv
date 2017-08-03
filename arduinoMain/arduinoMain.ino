
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "ServoAuv.h"
#include "OrientationSensor.h"

int errorFlag = 0;
OrientationSensor orient;
ServoAuv servo(1);
Gps gps(8,7);
Adafruit_GPS Ada = gps.getAdaGPS();
const int numPoints = 2;
Coord points[numPoints] = {Coord(60.203040, 24.961644), Coord(60.203197, 24.962177)};

void setup() 
{
  servo.begin();
  Serial.begin(19200);
  Serial.println("Begin setup");
//  while(true)
//  {
//    for(int i = -6; i <= 6; i++)
//    { 
//      Serial.println(i);
//      servo.turnTo(0); 
//      delay(500);
//    }
//  }
//  Coord x = gps.averageCoordinate(10);
//  Serial.println(x.latd);
  //orient.begin(errorFlag, true);

  
  Ada.begin(9200);
  // USE sendCommand() here. They don't seem to really work in the constructor of gps class
  // Also using it here once and then commenting it out might put the wanted code indefinitely
  Ada.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Ada.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //orient.saveCalibrationToEeprom(errorFlag);
//  while(!orient.isFullyCalibrated())
//  {
//    Serial.println("Calib:");
//    orient.displayCalStatus();
//    Serial.println(errorFlag);
//    delay(1000);
//  }
  
  Coord start(60.203264, 24.961356); // = gps.averageCoordinate(10);
  for (int i = 0; i < numPoints; i++)
  {
    Coord target = points[i];
    //Serial.println("before");
    Coord current = gps.averageCoordinate(10);
    //Serial.println(current.latd);
    float passDist = 5;
    float maxXte = 1E4;
    int bearing = current.bearingTo(target)*180/M_PI;
    while(current.distanceTo(target) >= passDist)
    {
      int newCourse = 0;
      float distToTarget, xte;
      bool inForbiddenZone;
      newBearing (start, target, current, passDist, maxXte, newCourse, distToTarget, xte, inForbiddenZone);

      holdCourse(gps, start, current, target, passDist, servo, newCourse, 7.0, 5.0, bearing, xte);     
    }
    //Serial.println("Arrived at WP");
    start = target;
  }
  Serial.println("Path Completed!");
  
}

void loop() 
{ 
  delay(100);
}

