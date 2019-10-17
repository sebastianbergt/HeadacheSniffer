#include <Arduino.h>
#include "MHZ19.h"

const int rx_pin = D2; //Serial rx pin no D2
const int tx_pin = D1; //Serial tx pin no D1
const int pwmpin = D5; //pwm pin no D5
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);
MHZ19 *mhz19_pwm = new MHZ19(pwmpin);

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  setup
  ----------------------------------------------------------*/
void setup()
{
    Serial.begin(115200);
    mhz19_uart->begin(rx_pin, tx_pin);
    mhz19_uart->setAutoCalibration(false);
    // while (mhz19_uart->isWarming())
    // {
    //     Serial.print("MH-Z19 now warming up...  status:");
    //     Serial.println(mhz19_uart->getStatus());
    //     //Serial.println( mhz19_pwm->getPPM(MHZ19_POTOCOL::PWM));
    //     delay(1000);
    // }
}

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  loop
  ----------------------------------------------------------*/
void loop()
{
    int co2ppm = mhz19_uart->getPPM(MHZ19_POTOCOL::UART);
    int temp = mhz19_uart->getTemperature();

    Serial.print("co2: ");
    Serial.println(co2ppm);
    Serial.print("temp: ");
    Serial.println(temp);

    // co2ppm = mhz19_pwm->getPPM(MHZ19_POTOCOL::PWM);
    // Serial.print("co2: ");
    // Serial.println(co2ppm);
    
    delay(5000);
}