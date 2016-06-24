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

#ifndef UTILITY_H_
#define UTILITY_H_

#include <math.h>
#include "auv.h"
#include "Coord.h"
class Coord;

/* void crossProduct (float v0[3], float v1[3], float n[3])
 * PURPOSE: Computes cross product of v0 x v1 and stores the result into n.
 */
void crossProduct (float v0[3], float v1[3], float n[3]);
	
void greatCircleVector (float heading, const Coord& current, float out[3]);

#ifdef SIMULATOR
  typedef unsigned char byte;
#endif

#if defined(ARDUINO)

#define millisecondTimer() millis()
#define waitUntil(statement, maxWait, timeout) \
        unsigned long startTime = millisecondTimer();\
        while(true) {\
        if (statement) {timeout = false; break;}\
        if ((((unsigned long)millisecondTimer()) - startTime > maxWait)) \
        {timeout = true; break;} \
        }
		
#elif defined(RASPBERRY_PI)
#include "time.h"
#define waitUntil(statement, maxWait, timeout) { \
        clock_t startTime = clock();\
        while(true) {\
        if (statement) {timeout = false; break;}\
        if (1000.0 * ((double)(clock() - startTime)) / CLOCKS_PER_SEC > maxWait) \
        {timeout = true; break;} \
        } \
}
#endif



#endif /* UTILITY_H_ */
