#ifndef GPRS_H_
#define GPRS_H_

#include <SoftwareSerial.h>

class Gprs
{
    private:
    SoftwareSerial _mySerial;
    String _avn;
    String _pin;

  public:
    explicit Gprs (String pin);

    void powerOn ();
    void powerOff ();
    void FlushSerialBuffer()
    String SubmitHttpRequest(String URL);

    
  private:
    String softwareSerialCurrentBufferToString();
    void setPin(String Pin);
    void pushPowerButton();
    //-----
    
};

#endif /* GPRS_H_ */
