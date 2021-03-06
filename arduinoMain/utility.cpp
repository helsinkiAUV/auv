/*
 * Utility functions for the AUV software.
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

#include "auv.h"
#include "utility.h"
#include "Coord.h"

void crossProduct (float v0[3], float v1[3], float n[3])
{
  float x0 = v0[0], y0 = v0[1], z0 = v0[2];
  float x1 = v1[0], y1 = v1[1], z1 = v1[2];
  n[0] = y0 * z1 - z0 * y1;
  n[1] = z0 * x1 - x0 * z1;
  n[2] = x0 * y1 - y0 * x1;
}

void greatCircleVector (float heading, const Coord& current, float out[3])
{
  out[0] = current.sinLon*cos(heading)-current.sinLat*current.cosLon*sin(heading); 
  out[1] = -current.cosLon*cos(heading)-current.sinLat*current.sinLon*sin(heading); 
  out[2] = current.cosLat*sin(heading);
}

float clamp(float minVal, float x, float maxVal)
{
  return fmax(minVal, fmin(x, maxVal));
}

