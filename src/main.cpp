#include <Arduino.h>
#include <WiFiClient.h>
#include "wifi_credentials.h" // const char* ssid = "ssid"; const char* password = "password";
#include "MHZ19.h"
const int rx_pin = D2; //Serial rx pin no D2
const int tx_pin = D1; //Serial tx pin no D1
const int pwmpin = D5; //pwm pin no D5
MHZ19 *mhz19_uart = new MHZ19(rx_pin, tx_pin);
MHZ19 *mhz19_pwm = new MHZ19(pwmpin);

void setup_wifi() {
  WiFi.begin(ssid, password);
  // Wait for connection
  Serial.println("  Connecting to ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  // connected
  Serial.println("");
  Serial.print("  connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void co2_sensor_setup() {
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(false);
  while(mhz19_uart->getStatus() < 0) {
    Serial.println("  MH-Z19B reply is not received or inconsistent.");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting setup...");
  setup_wifi();
  co2_sensor_setup();
  Serial.println("Setup done!");
}


void loop()
{
  int co2ppm = mhz19_uart->getPPM(MHZ19_POTOCOL::UART);
  int temp = mhz19_uart->getTemperature();

  Serial.print("co2: ");
  Serial.println(co2ppm);
  Serial.print("temp: ");
  Serial.println(temp);

  delay(5000);
}