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

#include "OrientationSensor.h"

float projectMagneticFieldToHorizontal (imu::Vector<3> reference_vec, imu::Vector<3> mag_meter_reading) {
   float norm = sqrt((mag_meter_reading.x()*mag_meter_reading.x()) + (mag_meter_reading.y()*mag_meter_reading.y()));
   mag_meter_reading.x() = mag_meter_reading.x() / norm;
   mag_meter_reading.y() = mag_meter_reading.y() / norm;
   return (360/PI)*atan2(mag_meter_reading.y() - reference_vec.y(), mag_meter_reading.x() - reference_vec.x()) + 180.0;
}

// RUN ONCE IN SETUP() WITHOUT INITIALIZING OrientationSensor OBJECT!
void saveCalibrationToEeprom(int& errorFlag)
{
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  if (!bno.begin()) errorStatus = ERROR_ORIENTATION_SENSOR_INITIALIZATION;
  Serial.println("Calibrate sensors (all of them!)");

  while (!bno.isFullyCalibrated())
  {
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    _bno.getCalibration(&system, &gyro, &accel, &mag);
   
    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
      Serial.print("! ");
    }
   
    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);
    delay(1500);
  }

  Serial.println("Sensor calibrated!");
  
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);

  int address = 0;
  bno.getSensor(&sensor);
  EEPROM.put(address, sensor.sensor_id);

  address += sizeof(long);
  EEPROM.put(address, newCalib);
  Serial.println("Data stored to EEPROM.");
}

void OrientationSensor::displayCalStatus()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  _bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

OrientationSensor::OrientationSensor (int& errorStatus) : _bno(55)
{
  if (!_bno.begin()) errorStatus = ERROR_ORIENTATION_SENSOR_INITIALIZATION;
}

imu::Vector<3> OrientationSensor::gravityField()
{
  //return _bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> ref_vec;
  ref_vec.x()=-1.0;
  ref_vec.y()=0.0;
  ref_vec.z()=0.0;
  return ref_vec;
}

imu::Vector<3> OrientationSensor::magneticField()
{
  return _bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

int OrientationSensor::heading()
{
  return (int) projectMagneticFieldToHorizontal(gravityField(), magneticField());
}

float OrientationSensor::temperature()
{
  return _bno.getTemp();
}


