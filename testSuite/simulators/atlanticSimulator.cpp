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
#include <string>
#include <sstream>
#include "Coord.h"
#include "GpsSimulator.h"
#include "RouteIo.h"
#include "navigation.h"

int main (int argc, char **argv)
{
//	Coord start(60,25);
//	Coord target(59.99, 25.01);
//	Coord current(60,25);
//
//	int newHeading;
//	float distToTarget, cte;
//	bool inForbiddenZone;
//	newBearing(start, target, current, 10, 5,
//				newHeading, distToTarget, cte, inForbiddenZone);
//
//	std::cout << newHeading << std::endl;
//	std::cout << distToTarget << std::endl;
//	std::cout << cte << std::endl;
//	std::cout << inForbiddenZone << std::endl;


	// let's read a config file
	// simulation file name is given as command line argument
	std::ifstream configFile(argv[1]);
	
	bool configFileLoaded = false;

	if(configFile) {
		configFileLoaded = true;
	}
	else {
		std::cout<<"There was a problem with opening simulation config file."<<std::endl;
		std::cout<<"Config file names are given as command line arguments:"<<std::endl;
		std::cout<<"> ./atlanticSimulator config.cfg "<<std::endl;
	}

	std::string fileString;
	std::string nameString,dataString;

	//simulation variables, leaving the default values still in
	std::ifstream inputFile("../../raspiMain/route/routeAtlantic.csv");
	std::string outputName = "../../raspiMain/route/routeAtlantic_simOut.csv";
	

	double boatSpeed = 0.8; // Speed of boat in m/s.
	double driftSpeed = 0.15; // Speed of unwanted drift [m/s] into the direction of wind.
	double windDir = 45; // DEGREES where the wind is blowing.
	double dt = 300; // Seconds;
	double heading = 45; // [0,360[;
	double GpsAccuracy = 5; // Meters
	double writeInterval = 3600; // Seconds.

	double simulationTime = 0; // Seconds.
	double simulationDistance = 0; // Meters.

	int numAver = 5; // Number of GPS averaging points

	//read the config file
	while(configFileLoaded && std::getline(configFile,fileString)) {
		if (!fileString.empty()) {
			if(fileString.at(0) != '#') {
				std::istringstream fileData(fileString);
				fileData >> nameString >> dataString;
				//handle the variables
				if(nameString == "name")
					std::cout<<"Using configuration file: "<<dataString<<std::endl;
				else if(nameString == "inputFile")
					std::ifstream inputFile(dataString);
				else if(nameString == "outputName")
					outputName == dataString;
				else if(nameString == "boatSpeed")
					boatSpeed = std::stod(dataString);
				else if(nameString == "driftSpeed")
					boatSpeed = std::stod(dataString);
				else if (nameString == "windDir")
					windDir = std::stod(dataString);
				else if (nameString == "dt")
					dt = std::stod(dataString);
				else if (nameString == "heading")
					heading = std::stod(dataString);
				else if (nameString == "GpsAccuracy")
					GpsAccuracy = std::stod(dataString);
				else if (nameString == "writeInterval")
					writeInterval = std::stod(dataString);
				else if (nameString == "simulationTime")
					simulationTime = std::stod(dataString);
				else if (nameString == "simulationDistance")
					simulationDistance = std::stod(dataString);
				else if (nameString == "numAver")
					numAver = std::stoi(dataString);
			}
		}
	}


	RouteIo routeIo (inputFile, outputName);

	/*
	std::ifstream inputFile("../../raspiMain/route/routeAtlantic.csv");
	std::string outputFile = "../../raspiMain/route/routeAtlantic_simOut.csv";
	RouteIo routeIo (inputFile, outputFile);

	double boatSpeed = 0.8; // Speed of boat in m/s.
	double driftSpeed = 0.15; // Speed of unwanted drift [m/s] into the direction of wind.
	double windDir = 45; // DEGREES where the wind is blowing.
	double dt = 300; // Seconds;
	double heading = 45; // [0,360[;
	double GpsAccuracy = 5; // Meters
	double writeInterval = 3600; // Seconds.

	double simulationTime = 0; // Seconds.
	double simulationDistance = 0; // Meters.

	int numAver = 5; // Number of GPS averaging points.
	*/

	Coord current = routeIo.getCurrentTargetCoord(); // Use first listed point as the initial point.
	Coord start = current;
	GpsSimulator gpsMock(current, boatSpeed, driftSpeed, windDir, dt, heading, GpsAccuracy);
	routeIo.moveToNextPoint(); // Switch to next point, where we head from the initial point.

	// Loop waypoints.
	while (!routeIo.reachedFinalPoint())
	{
		Coord target = routeIo.getCurrentTargetCoord();
		double maxCTE = std::max(routeIo.getCurrentMaxCTE(), 4 * GpsAccuracy);
		double maxPassDist = std::max(routeIo.getCurrentMaxPassDist(), 3 * GpsAccuracy);

		int outOfTrackPoints = 0;
		int maxOoTPoints = 100;
		current = gpsMock.averageCoordinate(numAver);
		Coord previous = current;

		double timeOfLastWrite = 0;

		// Main navigation loop running on Arduino.
		while (current.distanceTo(target) > maxPassDist && outOfTrackPoints < maxOoTPoints)
		{
			// Compute new heading.
			int newHeading;
			float distToTarget, cte;
			bool inForbiddenZone;
			newBearing(start, target, current, maxPassDist, maxCTE,
					newHeading, distToTarget, cte, inForbiddenZone);

			if (inForbiddenZone) outOfTrackPoints++; // Avoid infinite while loop in case of strong winds.

			if (simulationTime == 0.0 || simulationTime - timeOfLastWrite >= writeInterval)
			{
				std::ostringstream ss;
				ss << simulationTime;
				routeIo.write(ss.str(), current, cte, maxCTE);
				timeOfLastWrite = simulationTime;
			}
			simulationTime += dt;

			// Move the boat to next point.
			gpsMock.setHeading(newHeading);
			gpsMock.moveToNextPoint();
			previous = current;
			current = gpsMock.averageCoordinate(numAver);
			simulationDistance += current.distanceTo(previous);
		}

		if (outOfTrackPoints >= maxOoTPoints)
		{
			std::cout << "Way off course near waypoint: " << routeIo.currentWaypointNum() << std::endl;
		}

		start = target;
		routeIo.moveToNextPoint();
	}

	std::cout << "Distance (km): " << simulationDistance / 1000 << std::endl;
	std::cout << "Time (h): " << simulationTime / 3600 << std::endl;
	std::cout << "Average Speed (km/h): " << 3.6 * simulationDistance / simulationTime << std::endl;

	std::cout << "Output written to: " << outputName << std::endl;


	return 0;
}

