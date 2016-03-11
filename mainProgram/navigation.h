/*
 * Navigation functions.
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

/*
 * TODO:
 * - GPS-simulaattori (Ville).
 * - Funktion angularDistanceTo korjaus (Jami).
 * - Navigointifunktion kirjoittaminen (Juho).
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <math.h> // fmod, M_PI
double RE = 6378137; // Mean Earth radius.

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
  double fullCirc = 2.0 * M_PI;
  double halfCirc = M_PI;
  return fabs(fmod((fmod(b1-b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
}

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

  // Member functions (see documentation below).
  double bearingTo (const Coord& p) const;
  double distanceTo (const Coord& p) const;
  double angularDistanceTo (const Coord& p) const;
  bool leftOfTheGreatCircle (const Coord& start, const Coord& target) const;
  Coord closestGreatCirclePoint (const Coord& start, const Coord& target) const;
  void convertToXYZ (double xyz[3]) const;
};

/* Coord computeIntermediatePoint(double frac, double gcLength, const Coord& start
 *                              const Coord& target)
 * PURPOSE: Compute intermediate point at frac on a great circle between start and target.
 * INPUT:
 *     double frac     : Fraction [0,1] of distance traveled along the great circle.
 *     double gcLength : angular length of the great circle.
 *     Coord start     : Coordinates of the initial point (usually the previous waypoint).
 *     Coord target    : Coordinates of the target point.
 * OUTPUT:
 *     (Coord) coordinates of the intermediate point.
 */
Coord computeIntermediatePoint (double frac, double gcLength, const Coord& start, const Coord& target)
{
  double a = sin((1 - frac) * gcLength) / sin(gcLength);
  double b = sin(frac * gcLength) / sin(gcLength);
  double x = a * start.cosLat * start.cosLon + b * target.cosLat * target.cosLon;
  double y = a * start.cosLat * start.sinLon + b * target.cosLat * target.sinLon;
  double z = a * start.sinLat + b * target.sinLat;

  double pointLat = atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI;
  double pointLon = atan2(y, x) * 180.0 / M_PI;

  return Coord(pointLat, pointLon);
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

/* void crossProduct (double v0[3], double v1[3], double n[3])
 * PURPOSE: Computes cross product of v0 x v1 and stores the result into n.
 */
void crossProduct (double v0[3], double v1[3], double n[3])
{
  double x0 = v0[0], y0 = v0[1], z0 = v0[2];
  double x1 = v1[0], y1 = v1[1], z1 = v1[2];
  n[0] = y0 * z1 - z0 * y1;
  n[1] = z0 * x1 - x0 * z1;
  n[2] = x0 * y1 - y0 * x1;
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

// MAIN NAVIGATION FUNCTION:

/* void newBearing(...)
 * PURPOSE: Compute new bearing for the AUV.
 *          Remeber to check, whether the boat arrived at target before calling the function!
 * INPUT:
 *     Coord start             : Coordinates of the initial point (usually the previous waypoint).
 *     Coord target            : Coordinates of the target point.
 *     Coord current           : Present GPS coordinate.
 *     double maxPassDist      : Maximum acceptable distance for passing the target point [meters].
 *     double maxCrossTrackErr : Maximum distance from the great circle [meters].
 *  OUTPUT:
 *     double nextBearing      : New optimal bearing for the boat [DEGREES].
 *     double distToTarget     : Distance to target along a great circle (from current point to target) [meters].
 *     double crossTrackErr    : Current cross-track error [meters].
 *     bool inForbiddenZone    : Indicates, whether the boat is in a forbidden zone.
 */
void newBearing (const Coord& start, const Coord& target, const Coord& current, const double maxPassDist,
    const double maxCrossTrackErr, double& nextBearing, double& distToTarget, double& crossTrackErr, bool& inForbiddenZone)
{
  double r = current.distanceTo(target); // Distance from current point to target.
  double L = start.distanceTo(target); // Distance between waypoints.

  // Compute projection of the current point to the great circle.
  Coord projectionToGc = current.closestGreatCirclePoint(start, target);

  // Compute the y-coordinate.
  double pToTar = projectionToGc.distanceTo(target);
  double y = 0.0;
  // Check if we have accidentally gone past the target point, but are not within the maxPassDist.
  if (projectionToGc.distanceTo(start) >= L)
  {
    y = L + pToTar;
  }
  else
  {
    y = L - pToTar;
  }

  if (y >= L - maxPassDist) // If close to the target, approach target directly.
  {
    nextBearing = current.bearingTo(target) * 180 / M_PI;
    crossTrackErr = r;
    distToTarget = r;
    inForbiddenZone = true;
    return;
  }
  else
  {
    // Distance away from which the auv will begin approaching closer to the target.
    double appRange = fmax(L - 5 * maxPassDist, 0.0);
    double s; // Distance from the great circle to the boundary of the forbidden zone.
    if (y < appRange)
    {
      s = maxCrossTrackErr;
    }
    else // On approach to target.
    {
      // Slope of approach.
      double appSlope = (maxCrossTrackErr - maxPassDist) / (maxPassDist - appRange);
      s = maxPassDist + appSlope * (y - (L - maxPassDist));
    }

    double cte = current.distanceTo(projectionToGc); // Cross-track error.
    bool leftOfGc = current.leftOfTheGreatCircle(start, target);
    double bearingToTarget = current.bearingTo(target);
    double bearingToProjection = current.bearingTo(projectionToGc);

    // The larger the cross-track error, the steeper we approach the great circle.
    if (leftOfGc)
    {
      nextBearing = bearingToTarget + fmin((cte / s), 1.0) * angleBetweenBearings(bearingToProjection, bearingToTarget);
    }
    else
    {
      nextBearing = bearingToTarget - fmin((cte / s), 1.0) * angleBetweenBearings(bearingToProjection, bearingToTarget);
    }
    // Convert heading to degrees.
    nextBearing *= 180.0 / M_PI;
    if (nextBearing >= 360) nextBearing -= 360;
    if (nextBearing < 0) nextBearing += 360;

    distToTarget = r;
    crossTrackErr = cte;
    inForbiddenZone = cte > s;
    return;
  }

}

#endif // NAVIGATION_H_

