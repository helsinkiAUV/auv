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

/*
 * TODO:
 * - GPS-simulaattori (Ville).
 * - Funktion angularDistanceTo korjaus (Jami).
 * - Navigointifunktion kirjoittaminen (Juho).
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "Arduino.h"
#include "constants.h"
#include "Coord.h"
#include "Gps.h"
#include <math.h> // fmod, M_PI

// MAIN NAVIGATION FUNCTION:

/* void newBearing(...)
 * PURPOSE: Compute new bearing for the AUV.
 *          Remeber to check, whether the boat arrived at target before calling the function!
 * INPUT:
 *     Coord start             : Coordinates of the initial point (usually the previous waypoint).
 *     Coord target            : Coordinates of the target point.
 *     Coord current           : Present GPS coordinate.
 *     float maxPassDist       : Maximum acceptable distance for passing the target point [meters].
 *     float maxCrossTrackErr  : Maximum distance from the great circle [meters].
 *  OUTPUT:
 *     int nextBearing        : New optimal bearing for the boat [DEGREES].
 *     float distToTarget     : Distance to target along a great circle (from current point to target) [meters].
 *     float crossTrackErr    : Current cross-track error [meters].
 *     bool inForbiddenZone   : Indicates, whether the boat is in a forbidden zone. Forbidden zone is the region in which
 *                              the AUV is out of its navigational path (i.e. further away from the great circle than the
 *                              maxCrossTrackErr or, if accidentally passed the end point, maxPassDist).
 */
void newBearing (const Coord& start, const Coord& target, const Coord& current, const float maxPassDist,
                 const float maxCrossTrackErr, int& nextBearing, float& distToTarget, float& crossTrackErr, bool& inForbiddenZone);
#endif // NAVIGATION_H_
