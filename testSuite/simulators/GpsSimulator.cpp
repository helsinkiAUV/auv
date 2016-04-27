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
#include <ctime>
#include <random>


Coord GpsSimulator::getCurrentPoint() const
{
	return this->_currentPoint;
}

void GpsSimulator::setCurrentPoint(Coord point)
{
	this->_currentPoint = point;
}

Coord GpsSimulator::read () const
{   
	double pi = 3.1415926535897932384626433;
	
	std::mt19937 generator (time(NULL));
	std::uniform_real_distribution<double> randomNumber(0.0, 1.0);
	std::normal_distribution<double> randomUniform(0.0, _accuracy);

	double heading = 2.0*pi*randomNumber(generator);
	double distance = std::abs(randomUniform(generator));

	return _current.destination(heading, distance);
}

void GpsSimulator::moveToNextPoint ()
{   // Korjaa tamakin funktio. VINKKI: Coord::destination(heading, distance).
	//double _boatSpeed; // Speed of boat in m/s.
	//double _driftSpeed; // Speed of unwanted drift [m/s] into the direction of wind.
	//double _windDir; // RADIANS where the wind is blowing.
	//double _dt; // Seconds;

	Coord initialPoint = this->getCurrentPoint();	//if this is needed for the algorithm
	float heading = M_PI/2; //RAD
	float distance = 13; //meters

	this->setCurrentPoint(initialPoint.destination(heading,distance));
}
