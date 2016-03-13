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

/* void crossProduct (double v0[3], double v1[3], double n[3])
 * PURPOSE: Computes cross product of v0 x v1 and stores the result into n.
 */
void crossProduct (double v0[3], double v1[3], double n[3]);


#endif /* UTILITY_H_ */
