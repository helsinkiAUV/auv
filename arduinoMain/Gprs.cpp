#include "Gprs.h"
#include "Arduino.h"
//#include <SoftwareSerial.h>

// using flash memory to strings 3.8.2017
#include <avr/pgmspace.h>

Gprs::Gprs(String pin, int TX, int RX) : _gprsSerial(TX, RX)
{
      //_gprsSerial;
      _pin = pin;
      _apn = "internet.saunalahti";
 
      _gprsSerial.begin(19200);

}

String Gprs::softwareSerialCurrentBufferToString() 
{
   	String output = "";
        while(_gprsSerial.available()!=0)
	    output = output + _gprsSerial.read();
	
	return output;
}

void Gprs::FlushSerialBuffer()
{
    while(_gprsSerial.available()!=0)
        _gprsSerial.read();
}

void Gprs::powerOn() 
{
    pushPowerButton();
    delay(10000);
    //setPin(_pin);
    setPin();
}

void Gprs::powerOff() 
{
    pushPowerButton();
}

void Gprs::pushPowerButton()
{
    pinMode(9,OUTPUT);
    digitalWrite(9,LOW);
    delay(1000);
    digitalWrite(9,HIGH);
    delay(2000);
    digitalWrite(9,LOW);
}

//void Gprs::setPin(String Pin)
void Gprs::setPin() 
{
    String PINCall = "AT+CPIN=" + _pin;
   _gprsSerial.println(PINCall);
    delay(20000);
}

String Gprs::SubmitHttpRequest(String URL)
{
    Serial.println(F("SubmitHttp Start"));
    _gprsSerial.println("AT+CSQ");
    delay(100);
    Serial.println(_gprsSerial.read());
    _gprsSerial.println("AT+CGATT?");
    delay(100);
    _gprsSerial.println(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));//setting the SAPBR, the connection type is using gprs
    delay(1000);
    //String APNCall = "AT+SAPBR=3,1,\"APN\",\""+"internet.saunalahti"+"\"";
    String APNCall = "AT+SAPBR=3,1,\"APN\",\""+_apn+"\"";
    //String APNCall = "lol";
    _gprsSerial.println(APNCall);//setting the APN, the second need you fill in your local apn server
    delay(4000);
    _gprsSerial.println("AT+SAPBR=1,1");//setting the SAPBR, for detail you can refer to the AT command mamual
    delay(2000);
    _gprsSerial.println("AT+HTTPINIT"); //init the HTTP request
    delay(2000);
    String URLCall = "AT+HTTPPARA=\"URL\",\"" + URL + "\"";
    _gprsSerial.println(URLCall);// setting the httppara, the second parameter is the website you want to access
    delay(1000);

    String output;
    _gprsSerial.println("AT+HTTPACTION=0");//submit the request
    delay(10000);//the delay is very important, the delay time is base on the return from the website, if the return datas are very large, the time required longer.
    FlushSerialBuffer();

    _gprsSerial.println(F("AT+HTTPREAD=0,54"));// read the data from the website you access
    delay(300);
    for(int i = 0; i<15; i++) {
      _gprsSerial.read();
    }
    output = softwareSerialCurrentBufferToString();
    Serial.println(F("prel op"));
    Serial.println(output);    
    _gprsSerial.println("AT+HTTPREAD=55,54");// read the data from the website you access
    delay(300);
    for(int i = 0; i<16; i++) {
      _gprsSerial.read();
    }
    output = output + softwareSerialCurrentBufferToString();   
    delay(300);
    _gprsSerial.println("");
    delay(100);
    return output;
}
