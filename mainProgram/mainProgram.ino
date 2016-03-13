#include "Arduino.h"
#define FOR_ARDUINO
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"

// Sets up the environment for the main loop.
void setup() 
{
  Serial.begin(19200);
  Coord current(0.9, 24.9);
  Coord target(1, 25.0);
  Coord start(0, 25.0);

  int nh;
  double cte, dist;
  bool inForbZone;

  unsigned long t1, t2;
  t1 = micros();
  newBearing(start, target, current, 1E4, 1E5, nh, dist, cte, inForbZone);
  t2 = micros();
  
  Serial.println(nh);
  Serial.println(dist/1000, 1);
  //Serial.println(val, 10);
  Serial.println((t2-t1)/1000.0, 3);
}

void loop() 
{
  // put your main code here, to run repeatedly:

}
