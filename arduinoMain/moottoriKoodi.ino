#include "motor.hpp"

Motor moottori = Motor(7,5);
bool valueForLife = false;

void setup() {
  // put your setup code here, to run once:
  //digitalWrite(7,HIGH);
  Serial.begin(19200);
  moottori = Motor(7,5);
}

int i = 0;
void loop() {
  // put your main code here, to run repeatedly:
  //for(int i=0;i<255;i+=10) {
  //analogWrite(5,i);
  //delay(500);
  //Serial.println(i);
  //}
  //35 is pretty much zero, but still some torque

  //analogWrite(5,55);
  //analogWrite(5,i);
  //Serial.println("Signal: "+i); //ei toimi
  //if(i<255)
  //  i += 5;
  //delay(500);
  //Serial.println(moottori.getMotorSpeedValue());
  if(!valueForLife) {
    moottori.setMotorSpeed(0);
    moottori.updateMotorSpeed();
    Serial.println("jeejee");
  }
  Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  Serial.print("DigitalPort: ");
  Serial.println(moottori.getDigitalPort());
  Serial.print("AnalogPort: ");
  Serial.println(moottori.getAnalogPort());
  delay(5000);
  moottori.setMotorSpeed(-22);
  Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  delay(5000);
  moottori.setMotorSpeed(0);
  Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  delay(5000);
  moottori.setMotorSpeed(1);
  Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  delay(5000);
  moottori.setMotorSpeed(2);
    Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  delay(5000);
  moottori.setMotorSpeed(3);
  Serial.print("MotorSpeed: ");
  Serial.println(moottori.getMotorSpeedValue());
  delay(5000);
  valueForLife = true;
}
