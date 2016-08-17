#include "Arduino.h" // Include this to every header file, where you want to use the Arduino functions.
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include "navigation.h"


// Setup the GPS class
Gps gps(8,7);
Adafruit_GPS Ada = gps.getAdaGPS();


void setup() 
{
	Ada.begin(9600);
	Serial.begin(115200);

	Ada.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  	Ada.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  	delay(1000); //is this useless
}

// some code for testing
uint32_t timer = millis();
int time0 = millis();

void loop() 
{
	Coord here = gps.averageCoordinate(20);
	int tiem = millis();
	Serial.println(tiem-time0);
	time0 = tiem;
}
