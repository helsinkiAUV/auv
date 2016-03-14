/*
 * Navigation function.
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

#include "navigation.h"

void newBearing (const Coord& start, const Coord& target, const Coord& current, const double maxPassDist,
                 const double maxCrossTrackErr, int& nextBearing, double& distToTarget, double& crossTrackErr, bool& inForbiddenZone)
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
    nextBearing = (int) (current.bearingTo(target) * 180 / M_PI);
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
    double bearingToTarget = current.bearingTo(target);
    double bearingToProjection = current.bearingTo(projectionToGc);

    // The larger the cross-track error, the steeper we approach the great circle.
    // Correction is negative if on the right side of the great circle.
    double correctionAngle = fmin((cte / s), 1.0) * angleBetweenBearings(bearingToProjection, bearingToTarget);
    nextBearing = (int) ((bearingToTarget + correctionAngle) * 180.0 / M_PI);
    if (nextBearing >= 360) nextBearing -= 360;
    if (nextBearing < 0) nextBearing += 360;

    distToTarget = r;
    crossTrackErr = cte;
    inForbiddenZone = cte > s;
    return;
  }
}

