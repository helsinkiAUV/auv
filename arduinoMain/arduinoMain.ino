
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "ServoAuv.h"
#include "OrientationSensor.h"

int errorFlag = 0;
OrientationSensor orient;
ServoAuv servo(11);
Gps gps(8,7);
Adafruit_GPS Ada = gps.getAdaGPS();
const int numPoints = 3;
Coord points[numPoints] = {Coord(60.202371, 24.978190), Coord(60.20461, 24.979489), Coord(60.20713, 24.982666)};
//ServoTimer2 test;

void setup() 
{
  Serial.begin(19200);
  orient.begin(errorFlag, true);
  servo.begin();
  Ada.begin(9200);
  //test.attach(11);
//  Serial.println("Alive");
//  while(!orient.isFullyCalibrated())
//  {
//    Serial.println("Alive");
//    orient.displayCalStatus();
//    Serial.println(errorFlag);
//    delay(1000);
//  }

//    Coord start = gps.averageCoordinate(10);
//    for(int i = 0; i < numPoints; i++)
//    {
//      Coord current = gps.averageCoordinate(10);
//      Coord goal = points[i];
//      while (current.distanceTo(goal) > 25)
//      {
//        int nextBearing;
//        float r, cte;
//        bool inForbiddenZone;
//        newBearing (start, goal, current, 25,
//                   50, nextBearing, r, cte, inForbiddenZone);
//        Serial.print(current.latd); Serial.print(", ");
//        Serial.println(current.lond);
//        Serial.print("Course:"); Serial.println(nextBearing);
//        Serial.print("Heading:"); Serial.println(orient.heading());
//        Serial.print("r: "); Serial.println(r);
//        Serial.print("cte: "); Serial.println(cte);
//  
//        holdCourse(orient, servo, nextBearing, 10000, 2000, 100);
//        
//        Serial.println();
//        Coord current = gps.averageCoordinate(10);    
//      }
//      Serial.println("WP REACHED!");
//      start = points[i];
//    }
//    Serial.println("FINAL WP REACHED!");

  
}

void loop() 
{

  for (int i = -6; i <= 6; i++)
  {
    servo.turnTo(i);
//    delay(2000);
//      Coord current = gps.averageCoordinate(10);
//  Serial.println(current.latd);
    delay(1000);
  }
//  for(int i = 500; i < 2000; i++)
//  {
//    test.write(i); 
//    Serial.println(i);
//    delay(10);
//  }
//
//  servo.turnTo(0);


  //holdCourse(orient, servo, 90, 4000, 2000, 100);
  
  //servo.turnTo(random(-6,7));
  //Serial.println(orient.heading());
  
  delay(100);
}

