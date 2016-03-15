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

void newBearing (const Coord& start, const Coord& target, const Coord& current, const float maxPassDist,
                 const float maxCrossTrackErr, int& nextBearing, float& distToTarget, float& crossTrackErr, bool& inForbiddenZone)
{
  distToTarget = current.distanceTo(target); // Distance from current point to target.
  float distStartToTarget = start.distanceTo(target); // Distance between waypoints.

  // Compute projection of the current point to the great circle.
  Coord projectionToGc = current.closestGreatCirclePoint(start, target);

  float distProjToTarget = projectionToGc.distanceTo(target);
  float distStartToProj = 0.0;
  // Check if we have accidentally gone past the target point, but are not within the maxPassDist.
  if (projectionToGc.distanceTo(start) >= distStartToTarget)
  {
    distStartToProj = distStartToTarget + distProjToTarget;
  }
  else
  {
    distStartToProj = distStartToTarget - distProjToTarget;
  }

  if (distStartToProj >= distStartToTarget - maxPassDist) // If close to the target, approach target directly.
  {
    nextBearing = (int) (current.bearingTo(target) * 180 / M_PI);
    crossTrackErr = distToTarget;
    inForbiddenZone = true;
    return;
  }
  else
  {
    // Distance away from which the auv will begin approaching closer to the target.
    float approachRange = fmax(distStartToTarget - 5 * maxPassDist, 0.0);
    float distToForbidZoneFromGc; // Distance from the great circle (between start and target) to the boundary of the forbidden zone.
    if (distStartToProj < approachRange)
    {
      distToForbidZoneFromGc = maxCrossTrackErr;
    }
    else // On approach to target.
    {
      // Slope of approach.
      float appSlope = (maxCrossTrackErr - maxPassDist) / (maxPassDist - approachRange);
      distToForbidZoneFromGc = maxPassDist + appSlope * (distStartToProj - (distStartToTarget - maxPassDist));
    }

    crossTrackErr = current.distanceTo(projectionToGc); // Cross-track error.
    float bearingToTarget = current.bearingTo(target);
    float bearingToProjection = current.bearingTo(projectionToGc);

    // The larger the cross-track error, the steeper we approach the great circle.
    // Correction is negative if on the right side of the great circle.
    float correctionAngle = fmin((crossTrackErr / distToForbidZoneFromGc), 1.0) * angleBetweenBearings(bearingToProjection, bearingToTarget);
    nextBearing = (int) ((bearingToTarget + correctionAngle) * 180.0 / M_PI);
    if (nextBearing >= 360) nextBearing -= 360;
    if (nextBearing < 0) nextBearing += 360;

    inForbiddenZone = crossTrackErr > distToForbidZoneFromGc;
    return;
  }
}

