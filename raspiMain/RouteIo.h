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

/* class RouteIo
 * PURPOSE: Perform reading and writing of GPS waypoints.
 * INPUT FILE FORMAT:
 * 		latitude, longitude, maxCTE, maxPassDist [degrees, degrees, meters, meters]
 * OUTPUT FILE FORMAT:
 * 		YYYYMMDDHHMMSS, latitude, longitude, CTEratio [UTC time, degrees, degrees, unitless]
 */
class RouteIo
{
private:
	std::vector<Coord> _waypoints; // List of coordinates.
	std::vector<double> _maxCTEs; // Maximum cross-track errors.
	std::vector<double> _maxPassDists; // Waypoint radius.
	int _currentTarget; // Current target waypoint. One-based indexing!

	std::string _outputName; // Output file.

public:
	/* RouteIo (std::ifstream& inputFile, std::string outputName)
	 * PURPOSE: Constructor
	 * INPUT:
	 * 		ifstream& inputFile : Reference to an input file stream.
	 * 		string outputName   : Name of the output file.
	 */
	RouteIo (std::ifstream& inputFile, std::string outputName);

	/* 	Coord getCurrentTargetCoord() const
	 *	double getCurrentMaxCTE() const
	 *	double getCurrentMaxPassDist() const
	 *
	 *	PURPOSE: Get properties of the current target waypoint.
	 */
	Coord getCurrentTargetCoord() const;
	double getCurrentMaxCTE() const;
	double getCurrentMaxPassDist() const;

	/*  void moveToNextPoint()
	 *  Increase _currentTarget by one, thus moving to the next waypoint.
	 */
	void moveToNextPoint();

	/* 	Coord getTargetCoord(int waypointNum) const;
	 *  double getMaxCTE(int waypointNum) const;
	 *  double getMaxPassDist(int waypointNum) const;
	 *
	 *	PURPOSE: Get properties of a specific waypoint.
	 *	INPUT:
	 *		int waypointNum : Number of waypoint. ONE_BASED INDEXING!
	 *						  [1, numWaypoints()]!
	 */
	Coord getTargetCoord(int waypointNum) const;
	double getMaxCTE(int waypointNum) const;
	double getMaxPassDist(int waypointNum) const;

	/*  int numWaypoints() const
	 *  PURPOSE: Return the total number of waypoints.
	 */
	int numWaypoints() const;

	/*  int currentWaypointNum() const
	 *  PURPOSE: Return the current waypoint.
	 */
	int currentWaypointNum() const;

	/*  bool reachedFinalPoint() const
	 *  PURPOSE: Check whether we have reached the final waypoint.
	 *  		 Returns "true", if currentWaypointNum() > numWaypoints()
	 */
	bool reachedFinalPoint() const;

	/* void write (std::string timeStamp, Coord coordinate, double crossTrackErr, double maxCTE)
	 * PURPOSE: Write coordinate output to the file.
	 * INPUT:
	 * 		string timestamp     : Timestamp string as returned by getTimeString().
	 * 		Coord coordinate     : Coordinate point to write.
	 * 		double crossTrackErr : Cross-track error.
	 * 		double maxCTE        : Maximum cross-track error.
	 */
	void write (std::string timeStamp, Coord coordinate, double crossTrackErr, double maxCTE);
};


#endif /* ROUTEIO_H_ */
