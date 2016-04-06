/*
 * Main simulator software for the AUV.
 * Created by Juho Iipponen on April 6, 2016.
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

#include <iostream>
#include "Coord.h"
#include "GpsSimulator.h"

int main (int argc, char **argv)
{
	Coord current(60, 25);
	GpsSimulator gpsMock(current, 1.0, 0.5, 70, 100, 170, 10);

	gpsMock.moveToNextPoint();

	Coord next = gpsMock.read();

	std::cout << next << std::endl;

	return 0;
}

