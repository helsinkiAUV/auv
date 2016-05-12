/*
 * GPS simulator used in the testing.
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

#include <math.h>
#include "GpsSimulator.h"

Coord GpsSimulator::getCurrentPoint() const
{
	return this->_currentPoint;
}

void GpsSimulator::setCurrentPoint(Coord point)
{
	this->_currentPoint = point;
}

Coord GpsSimulator::read()
{
	std::uniform_real_distribution<double> randomUniform(0.0, 1.0);
	std::normal_distribution<double> randomNormal(0.0, _accuracy);

	double heading = 2.0 * M_PI * randomUniform(_generator);
	double distance = std::abs(randomNormal(_generator));

	return _currentPoint.destination(heading, distance);
}

void GpsSimulator::moveToNextPoint()
{
	double idealDistance = _boatSpeed * _dt; //s = v*t
	double driftDistance = _driftSpeed * _dt;

	//move the boat first to the ideal point disregarding the drift.
	Coord noDriftCoord = _currentPoint.destination(_heading, idealDistance);
	//and then to the point with drift
	Coord withDriftCoord = noDriftCoord.destination(_windDir + M_PI,
			driftDistance);
	_currentPoint = withDriftCoord;
}
