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

#ifndef SERVO_AUV_H_
#define SERVO_AUV_H_

#include "auv.h"
#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>

const int ANGLE_RANGE = 50;
const int NUM_STEPS = 6;
const int MAX_VAL = 590;
const int MIN_VAL = 190;

class ServoAuv
{
public:
  explicit ServoAuv(int channel, int minVal = MIN_VAL, int maxVal = MAX_VAL, 
                    int angleRange = ANGLE_RANGE, int numSteps = NUM_STEPS);
  void turnTo(int step);
  void begin();
  void detach();
  int getAngleRange() { return _angleRange; }
  int getNumSteps() { return _numSteps; }
  ~ServoAuv();
private:
  Adafruit_PWMServoDriver _arduinoServo;
  int _angleRange;
  int _numSteps;
  int _minVal;
  int _maxVal;
  int _channel;
};

#endif
