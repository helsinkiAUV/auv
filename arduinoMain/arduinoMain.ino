
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
Coord points[numPoints] = {Coord(60.320159, 25.083391), Coord(60.320754, 25.083293)};

void setup() 
{
  Serial.begin(19200);
  Serial.println("Begin setup");
  //orient.begin(errorFlag, true);

  servo.begin();
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
  Coord start = gps.averageCoordinate(10);
  for (int i = 0; i < numPoints; i++)
  {
    Coord target = points[i];
    Coord current = gps.averageCoordinate(10);

    float passDist = 10;
    float maxXte = 20;
    int bearing = current.bearingTo(target)*180/M_PI;
    while(current.distanceTo(target) >= passDist)
    {
      int newCourse = 0;
      float distToTarget, xte;
      bool inForbiddenZone;
      newBearing (start, target, current, passDist, maxXte, newCourse, distToTarget, xte, inForbiddenZone);
      Serial.println("----");
      Serial.print("Dist targ.: "); Serial.println(distToTarget);
      Serial.println("----");

      holdCourse(gps, current, target, passDist, servo, newCourse, 12.0, 5.0, bearing);     
    }
    Serial.println("Arrived at WP");
  }
  Serial.println("Path Completed!");
  
}

void loop() 
{ 
  delay(100);
}

