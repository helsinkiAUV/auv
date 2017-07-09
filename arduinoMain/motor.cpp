#include "motor.h"
#include "ARDUINO.H"

Motor::Motor(int p_digitalPort, int p_analogPort) {
  digitalPort = p_digitalPort;
  analogPort = p_analogPort;
  motorSpeedValue = 0;
  setUpArduino();
}

void Motor::stopMotor() {
  setMotorSpeed(0);
}

void Motor::setMotorSpeed(int p_motorSpeedValue) {
  if(p_motorSpeedValue >= 3) {
    motorSpeedValue = convertMotorSpeedValue(3);
  }
  else if (p_motorSpeedValue <= 0) {
    motorSpeedValue = convertMotorSpeedValue(0);
  }
  else {
    motorSpeedValue = convertMotorSpeedValue(p_motorSpeedValue);
  }
  updateMotorSpeed();
}

void Motor::setAnalogPort(int p_analogPort) {
  analogPort = p_analogPort;
}

void Motor::setDigitalPort(int p_digitalPort) {
  digitalPort = p_digitalPort;
}

void Motor::setUpArduino() {
  digitalWrite(getDigitalPort(),HIGH);
}

void Motor::updateMotorSpeed() {
  analogWrite(getAnalogPort(),getMotorSpeedValue());
}

int Motor::getDigitalPort() {
  return digitalPort;
}

int Motor::getAnalogPort() {
  return analogPort;
}

int Motor::getMotorSpeedValue() {
  return motorSpeedValue;
}

int Motor::convertMotorSpeedValue(int input) {
  if(input <= 0)
    return 0;
  else if (input == 1)
    return 220;
  else if (input == 2)
    return 245;
  else if (input == 3)
    return 255; 
}

//#include "motor.hpp"
//#include "Arduino.h"
//
//Motor::Motor(int p_digitalPort, int p_analogPort) {
//  digitalPort = p_digitalPort;
//  analogPort = p_analogPort;
//  motorSpeedValue = 0;
//  setUpArduino();
//}
//
//void Motor::stopMotor() {
//  setMotorSpeed(0);
//}
//
//void Motor::setMotorSpeed(int p_motorSpeedValue) {
//  if(p_motorSpeedValue >= 3) {
//    motorSpeedValue = convertMotorSpeedValue(3);
//  }
//  else if (p_motorSpeedValue <= 0) {
//    motorSpeedValue = convertMotorSpeedValue(0);
//  }
//  else {
//    motorSpeedValue = convertMotorSpeedValue(p_motorSpeedValue);
//  }
//  updateMotorSpeed();
//}
//
//void Motor::setAnalogPort(int p_analogPort) {
//  analogPort = p_analogPort;
//}
//
//void Motor::setDigitalPort(int p_digitalPort) {
//  digitalPort = p_digitalPort;
//}
//
//void Motor::setUpArduino() {
//  digitalWrite(getDigitalPort(),HIGH);
//}
//
//void Motor::updateMotorSpeed() {
//  analogWrite(getAnalogPort(),getMotorSpeedValue());
//}
//
//int Motor::getDigitalPort() {
//  return digitalPort;
//}
//
//int Motor::getAnalogPort() {
//  return analogPort;
//}
//
//int Motor::getMotorSpeedValue() {
//  return motorSpeedValue;
//}
//
//int Motor::convertMotorSpeedValue(int input) {
//  if(input <= 0)
//    return 0;
//  else if (input == 1)
//    return 220;
//  else if (input == 2)
//    return 245;
//  else if (input == 3)
//    return 255; 
//}


