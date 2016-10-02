/*
 * AUV Servo class.
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

 #include "ServoAuv.h"

ServoAuv::ServoAuv(int port, int minVal, int maxVal, int angleRange, int numSteps) : _angleRange(angleRange), _numSteps(numSteps)
{
   _arduinoServo.attach(port, minVal, maxVal);
}

void ServoAuv::turnTo(int step)
{
  _arduinoServo.write(90 + (float)_angleRange / _numSteps * step);
}

ServoAuv::~ServoAuv()
{
  _arduinoServo.detach();
}

