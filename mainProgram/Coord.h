/*
 * Coord struct.
 * Created by Juho Iipponen on March 12, 2016.
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

#ifndef COORD_H_
#define COORD_H_

#include "auv.h"
#include "constants.h"
#include "utility.h"
#include <math.h> // fmod, M_PI

/* struct Coord
 * PURPOSE: Stores a coordinate point and provides several helper functions to query the point.
 *          See member function documentation below.
 * EXAMPLE:
 *     Tee joku esimerkki.
 */
struct Coord
{
  float latd; // Latitude in decimal DEGREES.
  float lond; // Longitude in decimal DEGREES.

  float lat;  // Latitude in RADIANS.
  float lon;  // Longitude in RADIANS.

  float cosLat; // Cosine of latitude.
  float sinLat; // Sine of latitude.

  float cosLon; // Cosine of longitude.
  float sinLon; // Sine of longitude;

  // Constructor (input in degrees).
  Coord (float latIn, float lonIn) :
    latd(latIn), lond(lonIn), lat(latIn * M_PI / 180.0), lon(lonIn * M_PI / 180.0), cosLat(cos(lat)), sinLat(sin(lat)),
    cosLon(cos(lon)), sinLon(sin(lon))
  {
  }

  //Coord (const Coord& ) = default;
  //Coord (Coord&& ) = default;

  // Member functions.
  /* float Coord::bearingTo (const Coord& p)
   * PURPOSE: Computes initial bearing to point p.
   * INPUT:
   *     Coord p : Coordinate of another point.
   * OUTPUT:
   *     (float) initial bearing in RADIANS [0,2*pi[.
   */
  float bearingTo (const Coord& p) const;

  /* float Coord::distanceTo (const Coord& p) const
   * PURPOSE: Computes great circle distance to point p.
   * INPUT:
   *     Coord p : Coordinate of another point.
   * OUTPUT:
   *     (float) great circle distance to point p [meters].
   */
  float distanceTo (const Coord& p) const;

  /* float Coord::angularDistanceTo (const Coord& p) const
   * PURPOSE: Computes great circle angular distance to point p.
   * INPUT:
   *     Coord p : Coordinate of another point.
   * OUTPUT:
   *     (float) great circle angular distance to point p [RADIANS].
   */
  float angularDistanceTo (const Coord& p) const;

  /* Coord::leftOfTheGreatCircle (const Coord& start, const Coord& target) const
   * PURPOSE: Compute, wheter point is on the left or right side of the great circle
   *          as viewed from the start point to the target.
   * INPUT:
   *     Coord start   : Coordinates of the initial point (usually the previous waypoint).
   *     Coord target  : Coordinates of the target point.
   * OUTPUT:
   *     bool : true, if current point is on the left side; false if on the right.
   */
  bool leftOfTheGreatCircle (const Coord& start, const Coord& target) const;

  /* Coord Coord::closestGreatCirclePoint (const Coord& start, const Coord& target) const
   * PURPOSE: Computes the projection of the current point to the great circle
   *          between points start and target.
   * INPUT:
   *     Coord start   : Coordinates of the initial point (usually the previous waypoint).
   *     Coord target  : Coordinates of the target point.
   * OUTPUT:
   *     (Coord) coordinates of the closest point.
   *     NOTE: Can be outside the line segment from start to target.
   */
  Coord closestGreatCirclePoint (const Coord& start, const Coord& target) const;

  /* Coord Coord::destination (float heading, float distance)
   * PURPOSE: Compute the destination point given heading and distance from this. 
   *          COMPUTATION PERFORMED ALONG THE GREAT CIRCLE.
   * INPUT:
   *     float heading  : heading in RADIANS [0,2*pi[ from this.
   *     float distance : distance traveled in meters from this.
   * OUTPUT:
   *     (Coord) destination point.
   */
  Coord destination (float heading, float distance) const;

  /* void Coord::convertToXYZ(float xyz[3])
   * PURPOSE: Converts the coordinate to a cartesian system.
   * INPUT:
   *     float xyz[3] : Storage for output.
   * OUTPUT:
   *     float xyz[3] : Coordinate output.
   *
   */
  void convertToXYZ (float xyz[3]) const;

  /* Coord crossingPoint (float heading, Coord other, float otherHeading)
   * PURPOSE: To compute the crossing point between the current and some other
   *          great circle.
   * INPUT:
   *     float heading : current boat heading in RADIANS [0,2*pi[
   *     Coord other   : location of another body (e.g. another ship).
   *     float otherHeading : heading of the other body in RADIANS [0,2*pi[
   * OUTPUT:
   *     (Coord) The crossing point between the two great circles, which is 
   *             nearest to the current point.
   */
  Coord crossingPoint (float heading, Coord other, float otherHeading) const;
};

/* float angleBetweenBearings(const float b1, const float b2)
 * PURPOSE: Computes angle [0, pi] between two compass bearings.
 * INPUT:
 *     float b1 : bearing 1 (RADIANS)
 *     float b2 : bearing 2 (RADIANS)
 * OUTPUT:
 *     (float) angle between two bearings [0, pi]
 */
float angleBetweenBearings (const float b1, const float b2);

/* Coord convertToCoord (float xyz[3])
 * PURPOSE: Convert from cartesian to geodetic coordinates.
 * INPUT:
 *     float xyz[3] : cartesian coordinates
 * OUTPUT:
 *     (Coord) geodetic coordinates corresponding to xyz.
 *
 */
Coord convertToCoord (float xyz[3]);


#endif /* COORD_H_ */
