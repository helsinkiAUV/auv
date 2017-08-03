#ifndef GPRS_H_
#define GPRS_H_

#include <SoftwareSerial.h>

class Gprs
{
    private:
    SoftwareSerial _gprsSerial;
    String _apn;
    String _pin;

  public:
    explicit Gprs (String,int,int);

    void powerOn ();
    void powerOff ();
    void FlushSerialBuffer();
    String SubmitHttpRequest(String);

    
  private:
    String softwareSerialCurrentBufferToString();
    //void setPin(String);
    void setPin();
    void pushPowerButton();
    //-----
    
};

#endif /* GPRS_H_ */
