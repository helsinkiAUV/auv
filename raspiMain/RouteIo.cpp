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

#include "RouteIo.h"

RouteIo::RouteIo(std::ifstream& inputFile, std::string outputName)
{
	// Read the input file.
	std::string line;
	int lineNum = 0;

	if (inputFile.is_open())
	{
		while (std::getline(inputFile, line)) // Read line.
		{
			lineNum++;
			// Split line by comma ','.
			std::stringstream lineStream(line);
			std::string csvItem;
			std::vector<std::string> lineComponents;
			while (std::getline(lineStream, csvItem, ','))
			{
				lineComponents.push_back(csvItem);
			}
			try // Attempt to read the input row. If failed, the next row will be read.
			{
				if (lineComponents.size() == 4)
				{
					double lat = std::stod(lineComponents[0]);
					double lon = std::stod(lineComponents[1]);
					double maxCTE = std::stod(lineComponents[2]);
					double maxPassDist = std::stod(lineComponents[3]);

					_waypoints.push_back(Coord(lat, lon));
					_maxCTEs.push_back(maxCTE);
					_maxPassDists.push_back(maxPassDist);
				}
			} catch (...)
			{
			}
		}
		inputFile.close();
	}

	_currentTarget = 1;

	_outputName = outputName;
	// Remove possible existing file and test file creation.
	std::ofstream testOpen(outputName, std::ios::trunc);
	testOpen << "UTC,latitude,longitude,CTEratio" << std::endl;
	testOpen.close();
}

Coord RouteIo::getTargetCoord(int waypointNum) const
{
	return _waypoints[waypointNum - 1];
}

double RouteIo::getMaxCTE(int waypointNum) const
{
	return _maxCTEs[waypointNum - 1];
}

double RouteIo::getMaxPassDist(int waypointNum) const
{
	return _maxPassDists[waypointNum - 1];
}

Coord RouteIo::getCurrentTargetCoord() const
{
	return _waypoints[_currentTarget - 1];
}

double RouteIo::getCurrentMaxCTE() const
{
	return _maxCTEs[_currentTarget - 1];
}

double RouteIo::getCurrentMaxPassDist() const
{
	return _maxPassDists[_currentTarget - 1];
}

void RouteIo::moveToNextPoint()
{
	_currentTarget++;
}

int RouteIo::numWaypoints() const
{
	return (int) _waypoints.size();
}

int RouteIo::currentWaypointNum() const
{
	return _currentTarget; // One-based indexing!
}

bool RouteIo::reachedFinalPoint() const
{
	return currentWaypointNum() > numWaypoints();
}

void RouteIo::write (std::string timeStamp, Coord coordinate, double crossTrackErr, double maxCTE)
{
	std::ofstream outputFile(_outputName, std::ios::app); // Open file for appending.
	if (outputFile.fail()) // Appending operation failed.
	{ // TODO: Write this exception to the general error log.
		outputFile = std::ofstream(_outputName, std::ios::trunc); // Attempt to create a new file.
	}

	double CTEratio = crossTrackErr / maxCTE;

	if (outputFile.is_open())
	{
		// Always show exactly 8 digits after the decimal point and never use scientific notation.
		outputFile.setf( std::ios::fixed, std:: ios::floatfield );
		outputFile.precision(8);

		outputFile << timeStamp << ","
				   << coordinate.latd << ","
				   << coordinate.lond << ","
				   << CTEratio << std::endl;
		outputFile.close();
	}
	else
	{
		// TODO: Some very severe error!
	}


}

