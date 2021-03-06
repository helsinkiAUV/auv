/*
 * Constants for the AUV software.
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

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <math.h>
#include "auv.h"

const float RE = 6371E3; // Mean Earth radius.
#ifndef M_PI
  M_PI = 3.14159265358979323846_d
#endif
const float fullCirc = 2.0 * M_PI;
const float halfCirc = M_PI;

const int gpsNumOfAveragingPoints = 10;

#endif /* CONSTANTS_H_ */
