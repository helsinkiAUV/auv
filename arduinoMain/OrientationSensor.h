/*
 * Orientation sensor class.
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

#ifndef ORIENTATION_SENSOR_H_
#define ORIENTATION_SENSOR_H_

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Arduino.h"
#include <EEPROM.h>

#define BNO055_SAMPLERATE_DELAY_MS (100) // Sample rate
const int ERROR_ORIENTATION_SENSOR_INITIALIZATION = 1;
const int ERROR_ORIENTATION_SENSOR_CALIBRATION_TIMEOUT = 2;
const unsigned int MAX_CALIBRATION_TIME_MS = 10000;

float projectMagneticFieldToHorizontal (imu::Vector<3> reference_vec, imu::Vector<3> mag_meter_reading);


class OrientationSensor
{
  public:
  explicit OrientationSensor();
  void begin(int& errorStatus);
  int heading();
  float temperature();
  void displayCalStatus();
  imu::Vector<3> gravityField();
  imu::Vector<3> magneticField();
  void saveCalibrationToEeprom(int& errorFlag);
  void loadCalibrationData(int& errorFlag);
  bool isFullyCalibrated();
  void displaySensorStatus();
  void recalibrate(int& errorFlag);

  private:
  Adafruit_BNO055 _bno;
  adafruit_bno055_offsets_t _eepromReadCalibData;
};

#endif
