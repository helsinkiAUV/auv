#include "Gprs.h"
#include <SoftwareSerial.h>

Gprs::Gprs(int pin) 
{
      _mySerial;
      _pin = pin;
      _apn = "internet.saunalahti";
 
      _mySerial.begin(19200);

}

String softwareSerialCurrentBufferToString() 
{
   	String output = "";
        while(mySerial.available()!=0)
	    output = output + _mySerial.read();
	
	return output;
}

void FlushSerialBuffer()
{
    while(mySerial.available()!=0)
        mySerial.read();
}

void powerOn() 
}
    pushPowerButton();
    delay(10000);
    setPin(_pin);
}

void powerOff() 
}
    pushPowerButton();
}

void pushPowerButton()
{
    pinMode(9,OUTPUT);
    digitalWrite(9,LOW);
    delay(1000);
    digitalWrite(9,HIGH);
    delay(2000);
    digitalWrite(9,LOW);
}

void setPin(String Pin) 
{
    String PINCall = "AT+CPIN=" + PIN;
   _mySerial.println(PINCall);
    delay(20000);
}

String SubmitHttpRequest(String URL)
{
    _mySerial.println("AT+CSQ");
    delay(100);
    _mySerial.println("AT+CGATT?");
    delay(100);
    _mySerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");//setting the SAPBR, the connection type is using gprs
    delay(1000);
    String APNCall = "AT+SAPBR=3,1,\"APN\",\""+"internet.saunalahti"+"\"";
    _mySerial.println(APNCall);//setting the APN, the second need you fill in your local apn server
    delay(4000);
    _mySerial.println("AT+SAPBR=1,1");//setting the SAPBR, for detail you can refer to the AT command mamual
    delay(2000);
    _mySerial.println("AT+HTTPINIT"); //init the HTTP request
    delay(2000);
    String URLCall = "AT+HTTPPARA=\"URL\",\"" + URL + "\"";
    _mySerial.println(URLCall);// setting the httppara, the second parameter is the website you want to access
    delay(1000);

    String output;
    _mySerial.println("AT+HTTPACTION=0");//submit the request
    delay(10000);//the delay is very important, the delay time is base on the return from the website, if the return datas are very large, the time required longer.
    FlushSerialBuffer();

    mySerial.println("AT+HTTPREAD=0,54");// read the data from the website you access
    delay(300);
    for(int i = 0; i<15; i++) {
      mySerial.read();
    }
    output = softwareSerialCurrentBufferToString();    
    mySerial.println("AT+HTTPREAD=55,54");// read the data from the website you access
    delay(300);
    for(int i = 0; i<16; i++) {
      mySerial.read();
    }
    output = output + softwareSerialCurrentBufferToString();   
    delay(300);
    _mySerial.println("");
    delay(100);
    return output;
}
