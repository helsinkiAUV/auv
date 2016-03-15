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

#include "Coord.h"

double angleBetweenBearings (const double b1, const double b2)
{
  //return fabs(fmod((fmod(b1-b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
  return (fmod((fmod(b1-b2, fullCirc) + fullCirc + halfCirc), fullCirc) - halfCirc);
}

Coord convertToCoord (double xyz[3])
{
  double r = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
  double pointLat = 90 - (acos(xyz[2] / r) * 180.0 / M_PI);
  double pointLon = atan2(xyz[1], xyz[0]) * 180.0 / M_PI;
  return Coord(pointLat, pointLon);
}

Coord Coord::destination (double heading, double distance) const
{
  double angDist = distance / RE;
  double destLat = asin(sinLat * cos(angDist) + cosLat * sin(angDist) * cos(heading));
  double destLon = (lon + atan2(sin(heading)*sin(angDist)*cosLat, cos(angDist) - sinLat * sin(destLat)));
  destLat *= 180.0 / M_PI;
  destLon *= 180.0 / M_PI;
  return Coord(destLat, destLon);
}

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

double Coord::distanceTo (const Coord& p) const
{
  return angularDistanceTo(p) * RE;
}

double Coord::angularDistanceTo (const Coord& p) const
{ // TODO: Vaihda karteesisesta isoympyramitaksi.
// HUOMIOI, että Arduinossa double == float, joten
// numeeriseen epätarkkuuteen kannattaa kiinnittää huomiota!
  
    double a = sin((p.lat - lat)/2) * sin((p.lat - lat)/2) + cos(lon) * cos(p.lon) * sin((p.lon - lon)/2) * sin((p.lon - lon)/2);
    double greatCircleDistance = 2 * RE * atan2(sqrt(a), sqrt(1 - a));
    return greatCircleDistance;
}

bool Coord::leftOfTheGreatCircle (const Coord& start, const Coord& target) const
{
  // Bearing from start point to current point.
  double bearing_start_this = start.bearingTo(*this);
  // Bearing from start point to target.
  double bearing_start_target = start.bearingTo(target);
  return sin(bearing_start_this - bearing_start_target) < 0;
}

void Coord::convertToXYZ (double xyz[3]) const
{
  xyz[0] = cosLat * cosLon;
  xyz[1] = cosLat * sinLon;
  xyz[2] = sinLat;
  return;
}

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

