/*
 * Motor class.
 * Created by Väinö Katajisto on May 27, 2017.
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

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

//#include "auv.h"
//#include "constants.h"

class Motor
{
    private:
      int digitalPort; //turning motor on-off
      int analogPort; //for the motor signal
      int motorSpeedValue = 0;
      
    public:
      Motor (int,int);
      void stopMotor();
      void setMotorSpeed(int);
      void setDigitalPort(int);
      void setAnalogPort(int);
      void setUpArduino();
      void updateMotorSpeed();
      int getDigitalPort();
      int getAnalogPort();
      int getMotorSpeedValue();
      int convertMotorSpeedValue(int); //input the 0-3, output the arduino signal value
};

#endif /* MOTOR_HPP_ */
