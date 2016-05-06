/*
 * GPS route I/O.
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

#ifndef ROUTEIO_H_
#define ROUTEIO_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <ios>
#include <limits>
#include "Coord.h"

class RouteIo
{
private:
	std::vector<Coord> _waypoints;
	std::vector<double> _maxCTEs;
	std::vector<double> _maxPassDists;
	int _currentTarget; // One-based indexing!

	std::string _outputName;

public:
	RouteIo (std::ifstream& inputFile, std::string outputName);

	Coord getCurrentTargetCoord() const;
	double getCurrentMaxCTE() const;
	double getCurrentMaxPassDist() const;

	void moveToNextPoint();

	// One-based indexing.
	Coord getTargetCoord(int waypointNum) const;
	double getMaxCTE(int waypointNum) const;
	double getMaxPassDist(int waypointNum) const;

	int numWaypoints() const;
	int currentWaypointNum() const;

	bool reachedFinalPoint() const;

	void write (std::string timeStamp, Coord coordinate, double crossTrackErr, double maxCTE);
};


#endif /* ROUTEIO_H_ */
