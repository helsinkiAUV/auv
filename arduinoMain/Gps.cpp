/*
 * GPS class.
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

#include "auv.h"
#include "Gps.h"

Coord Gps::read()
{
  // KORJAA!
  return Coord(0, 0);
}

Coord Gps::averageCoordinate (int numPoints)
{
  Coord origin = read();
  if( numPoints == 1)
  {
    return origin; 
  }
 
  float meanX = 0;
  float meanY = 0;
  
  for(int i = 2; i <= numPoints; i++)
  { 
    Coord anotherPoint = read();
    float dist = origin.distanceTo(anotherPoint);                            
	float angle = origin.bearingTo(anotherPoint);	
	
	float x = dist * sin(angle);
	float y = dist * cos(angle);
	
	meanX += x/numPoints;
	meanY += y/numPoints;	
  }
  
  float averageDist = sqrt(meanX * meanX + meanY * meanY);
  float averageAngle = atan2(meanX,meanY);
  
  return origin.destination(averageAngle,averageDist);
  
}

int Gps::shutDown () const
{
  // KORJAA!
  return 0;
}

int Gps::turnOn () const
{
  // KORJAA!
  return 0;
}


