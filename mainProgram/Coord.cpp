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

#include "auv.h"
#include "Coord.h"

float angleBetweenBearings (const float b1, const float b2)
{
  //return fabs(fmod((fmod(b1-b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
  return (fmod((fmod(b1 - b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
}

Coord convertToCoord (float xyz[3])
{
  float r = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
  float pointLat = 90 - (acos(xyz[2] / r) * 180.0 / M_PI);
  float pointLon = atan2(xyz[1], xyz[0]) * 180.0 / M_PI;
  return Coord(pointLat, pointLon);
}

Coord Coord::destination (float heading, float distance) const
{
  float angDist = distance / RE;
  float destLat = asin(sinLat * cos(angDist) + cosLat * sin(angDist) * cos(heading));
  float destLon = (lon + atan2(sin(heading) * sin(angDist) * cosLat, cos(angDist) - sinLat * sin(destLat)));
  destLat *= 180.0 / M_PI;
  destLon *= 180.0 / M_PI;
  return Coord(destLat, destLon);
}

float Coord::bearingTo (const Coord& p) const
{
  float dy = sin(p.lon - lon) * p.cosLat;
  float dx = cosLat * p.sinLat - sinLat * p.cosLat * cos(p.lon - lon);
  float bear = fmod(atan2(dy, dx), 2.0 * M_PI);

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

float Coord::distanceTo (const Coord& p) const
{
  return angularDistanceTo(p) * RE;
}

float Coord::angularDistanceTo (const Coord& p) const
{
  float sinLatDiff = sin((p.lat - lat) / 2);
  float sinLonDiff = sin((p.lon - lon) / 2);
  float a = sinLatDiff * sinLatDiff + cosLat * p.cosLat * sinLonDiff * sinLonDiff;
  float greatCircleDistance = 2 * atan2(sqrt(a), sqrt(1 - a));
  return greatCircleDistance;
}

bool Coord::leftOfTheGreatCircle (const Coord& start, const Coord& target) const
{
  // Bearing from start point to current point.
  float bearing_start_this = start.bearingTo(*this);
  // Bearing from start point to target.
  float bearing_start_target = start.bearingTo(target);
  return sin(bearing_start_this - bearing_start_target) < 0;
}

void Coord::convertToXYZ (float xyz[3]) const
{
  xyz[0] = cosLat * cosLon;
  xyz[1] = cosLat * sinLon;
  xyz[2] = sinLat;
  return;
}

Coord Coord::closestGreatCirclePoint (const Coord& start, const Coord& target) const
{
  // Convert coordinate points to cartesian coordinates.
  float thisCoord[3];
  convertToXYZ(thisCoord);
  float startCoord[3];
  start.convertToXYZ(startCoord);
  float targetCoord[3];
  target.convertToXYZ(targetCoord);

  // Compute the normal vector for the plane given by start and target points (and origin).
  float n[3];
  crossProduct(startCoord, targetCoord, n);
  float nLen = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  // Normalize the normal vector.
  n[0] /= nLen;
  n[1] /= nLen;
  n[2] /= nLen;

  // Distance from the current point to the plane = dot(n, this)
  float d = thisCoord[0] * n[0] + thisCoord[1] * n[1] + thisCoord[2] * n[2];

  // Projection of the current point to the plane = this - d*n.
  float projectedPoint[3] = { thisCoord[0] - d * n[0], thisCoord[1] - d * n[1], thisCoord[2] - d * n[2] };
  return convertToCoord(projectedPoint);
}

Coord Coord::crossingPoint (float heading, Coord other, float otherHeading) const
{
  float greatCircleThis[3];
  float greatCircleOther[3];

  greatCircleVector(heading, *this, greatCircleThis);
  greatCircleVector(otherHeading, *this, greatCircleOther);

  float crossingPoint1[3];
  float crossingPoint2[3];
  
  crossProduct(greatCircleThis,greatCircleOther,crossingPoint1);
  crossProduct(greatCircleOther,greatCircleThis,crossingPoint2);
  
  Coord candidate1 = convertToCoord(crossingPoint1);
  Coord candidate2 = convertToCoord(crossingPoint2);
  
  if (distanceTo(candidate1) < distanceTo(candidate2))
  {
	  return candidate1;
  }
  else
  {
	  return candidate2;
  }
  
}