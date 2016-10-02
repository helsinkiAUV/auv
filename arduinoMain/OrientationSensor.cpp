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

// TODO: Remove all Serial.print()s and make the (re)calibration process automatic.
// TODO: Finish heading computation.

#include "OrientationSensor.h"
/* float computeHeadingFromHorizontalMagneticProjection (imu::Vector<3> gravityField, imu::Vector<3> magneticField) 
 * PURPOSE: Project the magnetic field to horizontal and thus compute the heading.
 * INPUT: 
 *     imu::Vector<3> gravityField : from a call to gravityField()
 *     imu::Vector<3> magneticField : from a call to magneticField()
 * OUTPUT:
 *     (float) heading in degrees [0,360].
 */
float computeHeadingFromHorizontalMagneticProjection (imu::Vector<3> gravityField, imu::Vector<3> magneticField) 
{
  imu::Vector<3> unitGravity = gravityField / gravityField.magnitude(); // Normalize gravity
  imu::Vector<3> horizontalMagnetic = magneticField - unitGravity.scale((magneticField.dot(unitGravity)));
  imu::Vector<3> relativeDirectionReference(-1,0,0); // Reference direction
  imu::Vector<3> horizontalReference = relativeDirectionReference - unitGravity.scale((relativeDirectionReference.dot(unitGravity)));

  float determinant = unitGravity.dot(horizontalReference.cross(horizontalMagnetic)); // Proportional to the sine of angle
  float dot = horizontalReference.dot(horizontalMagnetic); // Cosine of the angle between vectors
  float heading = atan2(determinant, dot)*180.0/M_PI + 180.0;
  return heading;
}

/* OrientationSensor::saveCalibrationToEeprom(int& errorFlag)
 * PURPOSE: Call once to save the current calibration state to EEPROM (non-volatile storage).
 * INPUT:
 *    int& errorFlag : Error status.
 */
void OrientationSensor::saveCalibrationToEeprom(int& errorFlag)
{
  while (!_bno.isFullyCalibrated()) // WARNING: Possible infinite loop.
  {
    Serial.println("Calibrate sensors (all states must be 3!)");
    displayCalStatus();
    delay(1500);
  }

  Serial.println("Sensor calibrated!");
  
  adafruit_bno055_offsets_t newCalib;
  _bno.getSensorOffsets(newCalib);

  int address = 0;
  sensor_t sensor;
  _bno.getSensor(&sensor);
  EEPROM.put(address, sensor.sensor_id);

  address += sizeof(long);
  EEPROM.put(address, newCalib);
  Serial.println("Data stored to EEPROM.");
}

/* OrientationSensor::loadCalibrationData(int& errorFlag)
 * PURPOSE: Load calibration data from EEPROM (non-volatile storage).
 * 
 */
void OrientationSensor::loadCalibrationData(int& errorFlag)
{
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  _bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("Could not find calibration data! Recalibration required.");
    saveCalibrationToEeprom(errorFlag);
  }

  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);

  _eepromReadCalibData = calibrationData;

  Serial.println("\n\nRestoring Calibration data to the BNO055...");
  _bno.setSensorOffsets(calibrationData);

  sensors_event_t event;
  _bno.getEvent(&event);
  if (!_bno.isFullyCalibrated()){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!_bno.isFullyCalibrated())
      {
          _bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  Serial.println("All Orientation sensors calibrated!");
}

/* OrientationSensor::displayCalStatus()
 * PURPOSE: Display calibration status in pretty print.
 */
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

/* OrientationSensor::OrientationSensor ()
 * PURPOSE: Constructor. begin() (see below) must be called before the sensor can be used!
 */
OrientationSensor::OrientationSensor () : _bno(55), _eepromReadCalibData(adafruit_bno055_offsets_t())
{
}

/* OrientationSensor::recalibrate(int& errorFlag)
 * PURPOSE: Recalibrate the magnetic field sensor and set gyro/acc to EEPROM read defaults.
 */
void OrientationSensor::recalibrate(int& errorFlag)
{
  _bno.setSensorOffsets(_eepromReadCalibData);
  unsigned long t0 = millis();
  
  sensors_event_t event;
  _bno.getEvent(&event);
  if (!_bno.isFullyCalibrated()){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!_bno.isFullyCalibrated() && millis() - t0 < MAX_CALIBRATION_TIME_MS)
      {
          _bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  if (millis() - t0 >= MAX_CALIBRATION_TIME_MS)
  {
    errorFlag = ERROR_ORIENTATION_SENSOR_CALIBRATION_TIMEOUT;
    return;
  }
  
  Serial.println("Recalibration complete");
}

/* OrientationSensor::begin(int& errorStatus)
 * PURPOSE: Begin communicating with the sensor and load the calibration data.
 */
void OrientationSensor::begin(int& errorStatus)
{
  if (!_bno.begin()) 
  {
    errorStatus = ERROR_ORIENTATION_SENSOR_INITIALIZATION;
    return;
  }
  loadCalibrationData(errorStatus);
}

/* OrientationSensor::isFullyCalibrated()
 * PURPOSE: Check whether all calibration status are "3".
 */
bool OrientationSensor::isFullyCalibrated()
{
  return _bno.isFullyCalibrated();
}

/* OrientationSensor::displaySensorStatus()
 * PURPOSE: Display status.
 */
void OrientationSensor::displaySensorStatus()
{
  return _bno.displaySystemStatus();
}

/* imu::Vector<3> OrientationSensor::gravityField()
 * PURPOSE: Output the gravitational field of the Earth.
 * OUTPUT:
 *    imu::Vector<3> : the three-dimensional gravity field in the sensor reference-frame. [m/s^2].
 */
imu::Vector<3> OrientationSensor::gravityField()
{
  return _bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
}

/* imu::Vector<3> OrientationSensor::magneticField()
 * PURPOSE: Output the magnetic field of the Earth.
 * OUTPUT:
 *    imu::Vector<3> : the three-dimensional magnetic field in the sensor reference-frame. [uT].
 */
imu::Vector<3> OrientationSensor::magneticField()
{
  return _bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

/* int OrientationSensor::heading()
 * PURPOSE: Compute the heading of the boat.
 * OUTPUT:
 *    (int) heading : Orientation-corrected heading [0,359].
 */
int OrientationSensor::heading()
{
  return (int) computeHeadingFromHorizontalMagneticProjection(gravityField(), magneticField());
}

/* float OrientationSensor::temperature()
 * PURPOSE: Output temperature measured by the on-board chip.
 * OUTPUT:
 *    (float) temperature : Temperature in degrees Celsius.
 */
float OrientationSensor::temperature()
{
  return _bno.getTemp();
}
