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

Coord GpsSimulator::read () const
{   // Korjaa tama funktio.
	return Coord(1,1);
}

void GpsSimulator::moveToNextPoint ()
{   
	Coord initialPoint = this->getCurrentPoint();
	double boatDirection = this->getHeading();
	double windDirection = this->getWindDir();
	double idealDistance = this->getBoatSpeed()*this->getDt(); //s = v*t
	double driftDistance = this->getDriftSpeed()*this->getDt();

	//move the boat first to the ideal point
	this->setCurrentPoint(initialPoint.destination(boatDirection,idealDistance));
	//and then to the point with drift
	this->setCurrentPoint(initialPoint.destination(windDirection,driftDistance));
}





