/*
 * GPS class.
 * Created by Juho Iipponen on March 11, 2016.
 *
 * This file is part of the University of Helsinki AUV source code.
 *
 * The AUV source code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * The AUV source code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the source code.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#ifndef GPS_H_
#define GPS_H_

#include "auv.h"
#include "Coord.h"
#include "constants.h"

 #include "Adafruit_GPS.h"
 #include <SoftwareSerial.h>

class Gps
{
    private:
    SoftwareSerial _gpsSerial;
    Adafruit_GPS _AdaGPS;

  public:
    explicit Gps (int, int);


   /* Coord Gps::averageCoordinate () const
    * PURPOSE: Return weigted average coordinate, which represents the best guess of the current position.
    */
    Coord averageCoordinate (int numPoints);

    /* virtual Coord Gps::read()
     * PURPOSE: Reads in the current GPS coordinate.
     * NOTE: Overloaded in testSuite's GpsSimulator
     */
    virtual Coord read ();

    int shutDown () const;

    int turnOn () const;

    //-----
    Adafruit_GPS getAdaGPS() {return _AdaGPS; };
    SoftwareSerial getGpsSerial() {return _gpsSerial; };
    
};

#endif /* GPS_H_ */
