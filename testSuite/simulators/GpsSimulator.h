/*
 * GPS simulator used in the testing.
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

#ifndef GPSSIMULATOR_H_
#define GPSSIMULATOR_H_

#include "Coord.h"
#include "Gps.h"
#include <math.h>

class GpsSimulator : public Gps
{
public:
	GpsSimulator (Coord currentPoint, double bs, double ds, int wd, double dt, int heading, double acc) :
			_currentPoint(currentPoint), _boatSpeed(bs), _driftSpeed(ds), _windDir(wd * M_PI / 180.0), _dt(dt), _heading(
					heading * M_PI / 180.0), _accuracy(acc)
	{
	}

	/* Coord GpsSimulator::read() const
	 * PURPOSE: To give gps coordinates randomized in the vicinity of the currentPoint point.
	 *          Random distribution only affected by _accuracy.
	 * OUTPUT:
	 *     (Coord) random reading.
	 */
	Coord read () const;

	/* void GpsSimulator::moveToNextPoint()
	 * PURPOSE: Move to a next point along the constant heading, taking account the drift.
	 */
	void moveToNextPoint ();

	// Getters and setters:

	double getAccuracy () const
	{
		return _accuracy;
	}

	void setAccuracy (double accuracy)
	{
		_accuracy = accuracy;
	}

	double getBoatSpeed () const
	{
		return _boatSpeed;
	}

	void setBoatSpeed (double boatSpeed)
	{
		_boatSpeed = boatSpeed;
	}

	double getDriftSpeed () const
	{
		return _driftSpeed;
	}

	void setDriftSpeed (double driftSpeed)
	{
		_driftSpeed = driftSpeed;
	}

	double getDt () const
	{
		return _dt;
	}

	void setDt (double dt)
	{
		_dt = dt;
	}

	int getHeading () const
	{
		return _heading;
	}

	void setHeading (int heading)
	{
		_heading = heading;
	}

	int getWindDir () const
	{
		return _windDir;
	}

	void setWindDir (int windDir)
	{
		_windDir = windDir;
	}

private:
	Coord _currentPoint; // Current point.
	double _boatSpeed; // Speed of boat in m/s.
	double _driftSpeed; // Speed of unwanted drift [m/s] into the direction of wind.
	double _windDir; // RADIANS where the wind is blowing.
	double _dt; // Seconds;
	double _heading; // [0,2*pi[;
	double _accuracy; // Meters
};

#endif /* GPSSIMULATOR_H_ */
