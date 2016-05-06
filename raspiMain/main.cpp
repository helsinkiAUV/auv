/*
 * Raspberry pi main function.
 * Created by Juho Iipponen on May 5, 2016.
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

#include "RouteIo.h"
#include <iostream>
#include <fstream>
#include "timeString.h"

#ifndef SIMULATOR
int main ()
{
	std::ifstream inputFile("route/routeArabianranta.csv");
	std::string outputFile("route/routeOut.csv");

	RouteIo routeIo (inputFile, outputFile);

	routeIo.write(getTimeString(), routeIo.getCurrentTargetCoord(), 5, 10);

	return 0;
}
#endif
