
#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"
#include "ServoAuv.h"
#include "OrientationSensor.h"
#include "Gprs.h"

int errorFlag = 0;
OrientationSensor orient;
ServoAuv servo(1);
Gps gps(8,7);
Adafruit_GPS Ada = gps.getAdaGPS();
//const int numPoints = 3;
//Coord points[numPoints] = {Coord(60.202371, 24.978190), Coord(60.20461, 24.979489), Coord(60.20713, 24.982666)};

void setup() 
{
  Serial.begin(19200);
  Serial.println("Begin setup");
  //orient.begin(errorFlag, true);
  //servo.begin();
  Ada.begin(9200);
  // USE sendCommand() here. They don't seem to really work in the constructor of gps class
  // Also using it here once and then commenting it out might put the wanted code indefinitely
  Ada.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Ada.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //orient.recalibrate(errorFlag);
  //while(!orient.isFullyCalibrated())
  //{
  //  Serial.println("Calib:");
  //  orient.displayCalStatus();
  //  Serial.println(errorFlag);
  //  delay(1000);
  //}


  double A = 5.0; //metriä //säde jonka ulkopuolella informaatio ylittää kohinan
  float heading = 0;
  double distance = 0.0;
  Coord goalPoint = Coord(60.202997,24.961732);
  float goalDistance = 0.0;
  float goalHeading = 0.0;
  // HEADING TESTI 5.7.2017
  delay(5000);
    Coord origin = gps.averageCoordinate(10); //alkupiste
    Coord current = gps.averageCoordinate(10);
    
  while(true)
  {
    Serial.println("Calculating distance to goal");
    goalDistance = current.distanceTo(goalPoint);
    goalHeading = current.bearingTo(goalPoint)*180/M_PI;
    Serial.print("Distance to goal (m): ");Serial.println(goalDistance);
    Serial.print("Heading to goal: ");Serial.println(goalHeading);
    while(true) {
      //lasketaan etäisyys
      current = gps.averageCoordinate(10);
      distance = origin.distanceTo(current); // etäisyys nykypisteeseen
      Serial.print("d(origin,current) = ");Serial.println(distance);
      //Serial.print(origin.latd,6);Serial.print(",");Serial.println(origin.lond,6);
      //Serial.print(current.latd,6);Serial.print(",");Serial.println(current.lond,6);
      Serial.println("-----");
      Serial.println("Calculating distance to goal");
      goalDistance = current.distanceTo(goalPoint);
      goalHeading = current.bearingTo(goalPoint)*180/M_PI;
      Serial.print("Distance to goal (m): ");Serial.println(goalDistance);
      Serial.print("Heading to goal: ");Serial.println(goalHeading);
      Serial.println("-----");
      if(distance >= A) {
        Serial.println("Lasketaan heading, koska distance > A = 10.0 m");
        heading = origin.bearingTo(current)*180/M_PI;
        Serial.print("Heading: ");Serial.println(heading);
        origin = current;
        break;
      }
    }
    
    //Coord current = gps.averageCoordinate(10); 
    //int heading = orient.heading();
    //Serial.println("average coordinate:");
    //Serial.print(current.latd,6);Serial.print(",");Serial.println(current.lond,6);
    //Serial.println(heading);
    //Serial.println();
  }
  
}

void loop() 
{
//  Serial.println("Begin loop");
//  for (int i = -6; i <= 6; i++)
//  {
//    servo.turnTo(i);
////    delay(2000);
////      Coord current = gps.averageCoordinate(10);
//    Serial.println(i);
//    delay(1000);
//  }
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

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

//#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
//
//// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//// you can also call it with a different address you want
////Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
//
//// Depending on your servo make, the pulse width min and max may vary, you 
//// want these to be as small/large as possible without hitting the hard stop
//// for max range. You'll have to tweak them as necessary to match the servos you
//// have!
//#define SERVOMIN  700 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  2300 // this is the 'maximum' pulse length count (out of 4096)
//
//// our servo # counter
//uint8_t servonum = 0;
//int pos = 2000;
//
//void setup() {
//  Serial.begin(9600);
//  Serial.println("16 channel Servo test!");
//
//  pwm.begin();
//  
//  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
//
//  yield();
//}
//
//void loop() 
//{
//    Serial.println("Give position: ");
//    String stat = Serial.readString();
//    if (stat.toInt() != pos && stat.toInt() > 0) pos = stat.toInt();
//    Serial.print("Pos: ");
//    Serial.println(pos);
//    pwm.setPWM(1, 0, pos);
//    delay(1000);
//
//}

