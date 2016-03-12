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
  double latd; // Latitude in decimal DEGREES.
  double lond; // Longitude in decimal DEGREES.

  double lat;  // Latitude in RADIANS.
  double lon;  // Longitude in RADIANS.

  double cosLat; // Cosine of latitude.
  double sinLat; // Sine of latitude.

  double cosLon; // Cosine of longitude.
  double sinLon; // Sine of longitude;

  // Constructor (input in degrees).
  Coord (double latIn, double lonIn) :
      latd(latIn), lond(lonIn), lat(latIn * M_PI / 180.0), lon(lonIn * M_PI / 180.0), cosLat(cos(lat)), sinLat(sin(lat)), cosLon(
          cos(lon)), sinLon(sin(lon))
  {
  }

  Coord (const Coord& ) = default;
  Coord (Coord&& ) = default;

  // Member functions (see documentation below).
  double bearingTo (const Coord& p) const;
  double distanceTo (const Coord& p) const;
  double angularDistanceTo (const Coord& p) const;
  bool leftOfTheGreatCircle (const Coord& start, const Coord& target) const;
  Coord closestGreatCirclePoint (const Coord& start, const Coord& target) const;
  Coord destination (double heading, double distance) const;
  void convertToXYZ (double xyz[3]) const;
};

/* double angleBetweenBearings(const double b1, const double b2)
 * PURPOSE: Computes angle [0, pi] between two compass bearings.
 * INPUT:
 *     double b1 : bearing 1 (RADIANS)
 *     double b2 : bearing 2 (RADIANS)
 * OUTPUT:
 *     (double) angle between two bearings [0, pi]
 */
double angleBetweenBearings (const double b1, const double b2)
{
  return fabs(fmod((fmod(b1-b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
}

/* Coord Coord::destination (double heading, double distance)
 * PURPOSE: Compute destination point given heading and distance from this.
 * INPUT:
 *     double heading  : heading in RADIANS [0,2*pi[
 *     double distance : distance traveled in meters.
 * OUTPUT:
 *     (Coord) destination point.
 */
Coord Coord::destination (double heading, double distance) const
{
	double angDist = distance / RE;
	double destLat = asin(sinLat * cos(angDist) + cosLat * sin(angDist) * cos(heading));
	double destLon = (lon + atan2(sin(heading)*sin(angDist)*cosLat, cos(angDist) - sinLat * sin(destLat)));
	destLat *= 180.0 / M_PI;
	destLon *= 180.0 / M_PI;
	return Coord(destLat, destLon);
}

// MEMBERS OF Coord:

/* double Coord::bearingTo (const Coord& p)
 * PURPOSE: Computes initial bearing to point p.
 * INPUT:
 *     Coord p : Coordinate of another point.
 * OUTPUT:
 *     (double) initial bearing in RADIANS [0,2*pi[.
 */
double Coord::bearingTo (const Coord& p) const
{
  double dy = sin(p.lon - lon) * p.cosLat;
  double dx = cosLat * p.sinLat - sinLat * p.cosLat * cos(p.lon - lon);
  double bear = fmod(atan2(dy, dx), 2.0 * M_PI);

  if (bear >= 2.0 * M_PI)
  {
    bear -= 2.0 * M_PI;
  }
  if (bear < 0)
  {
    bear += 2.0 * M_PI;
  }
  return bear;
}

/* double Coord::distanceTo (const Coord& p)
 * PURPOSE: Computes great circle distance to point p.
 * INPUT:
 *     Coord p : Coordinate of another point.
 * OUTPUT:
 *     (double) great circle distance to point p [meters].
 */
double Coord::distanceTo (const Coord& p) const
{
  return angularDistanceTo(p) * RE;
}

/* double Coord::angularDistanceTo (const Coord& p)
 * PURPOSE: Computes great circle angular distance to point p.
 * INPUT:
 *     Coord p : Coordinate of another point.
 * OUTPUT:
 *     (double) great circle angular distance to point p [RADIANS].
 */
double Coord::angularDistanceTo (const Coord& p) const
{ // TODO: Vaihda karteesisesta isoympyramitaksi.
// HUOMIOI, että Arduinossa double == float, joten
// numeeriseen epätarkkuuteen kannattaa kiinnittää huomiota!
  double dx = cos((lat + p.lat) * 0.5) * (p.lon - lon);
  double dy = (p.lat - lat);
  return sqrt(dy * dy + dx * dx);
}

/* Coord::leftOfTheGreatCircle (const Coord& start, const Coord& target) const
 * PURPOSE: Compute, wheter point is on the left or right side of the great circle
 *          as viewed from the start point to the target.
 * INPUT:
 *     Coord start   : Coordinates of the initial point (usually the previous waypoint).
 *     Coord target  : Coordinates of the target point.
 * OUTPUT:
 *     bool : true, if current point is on the left side; false if on the right.
 */
bool Coord::leftOfTheGreatCircle (const Coord& start, const Coord& target) const
{
  // Bearing from start point to current point.
  double bearing_start_this = start.bearingTo(*this);
  // Bearing from start point to target.
  double bearing_start_target = start.bearingTo(target);
  return sin(bearing_start_this - bearing_start_target) < 0;
}

/* void Coord::convertToEcef(double xyz[3])
 * PURPOSE: Converts the coordinate to a cartesian system.
 * INPUT:
 *     double xyz[3] : Storage for output.
 * OUTPUT:
 *     double xyz[3] : Coordinate output.
 *
 */
void Coord::convertToXYZ (double xyz[3]) const
{
  xyz[0] = cosLat * cosLon;
  xyz[1] = cosLat * sinLon;
  xyz[2] = sinLat;
  return;
}

/* Coord convertToCoord (double xyz[3])
 * PURPOSE: Convert from cartesian to geodetic coordinates.
 * INPUT:
 *     double xyz[3] : cartesian coordinates
 * OUTPUT:
 *     (Coord) geodetic coordinates corresponding to xyz.
 *
 */
Coord convertToCoord (double xyz[3])
{
  double r = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
  double pointLat = 90 - (acos(xyz[2] / r) * 180.0 / M_PI);
  double pointLon = atan2(xyz[1], xyz[0]) * 180.0 / M_PI;
  return Coord(pointLat, pointLon);
}

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
Coord Coord::closestGreatCirclePoint (const Coord& start, const Coord& target) const
{
  // Convert coordinate points to cartesian coordinates.
  double thisCoord[3];
  convertToXYZ(thisCoord);
  double startCoord[3];
  start.convertToXYZ(startCoord);
  double targetCoord[3];
  target.convertToXYZ(targetCoord);

  // Compute the normal vector for the plane given by start and target points (and origin).
  double n[3];
  crossProduct(startCoord, targetCoord, n);
  double nLen = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  // Normalize the normal vector.
  n[0] /= nLen;
  n[1] /= nLen;
  n[2] /= nLen;

  // Distance from the current point to the plane = dot(n, this)
  double d = thisCoord[0] * n[0] + thisCoord[1] * n[1] + thisCoord[2] * n[2];

  // Projection of the current point to the plane = this - d*n.
  double projectedPoint[3] = { thisCoord[0] - d * n[0], thisCoord[1] - d * n[1], thisCoord[2] - d * n[2] };
  return convertToCoord(projectedPoint);
}



#endif /* COORD_H_ */
