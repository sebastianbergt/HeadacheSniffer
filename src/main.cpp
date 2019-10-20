#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "wifi_credentials.h"   // const char* ssid = "ssid"; const char* password = "password";
#include "server_credentials.h" // const char* host = "google.com"; const uint16_t port = 80;
#include "MHZ19.h"

const int rx_pin = D2; //Serial rx pin no D2
const int tx_pin = D1; //Serial tx pin no D1
MHZ19 *mhz19_uart = new MHZ19(rx_pin, tx_pin);

void setup_wifi()
{
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

void co2_sensor_setup()
{
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(false);
  while (mhz19_uart->getStatus() < 0)
  {
    Serial.println("  MH-Z19B reply is not received or inconsistent.");
    delay(100);
  }
}

void prepareSchema(const measurement_t &m_in, std::string &data)
{
  using namespace ArduinoJson;
  // example ~ 68 byte => {"v":1,"mac":"00:00:00:00:00:00","co2":2000,"temp":10.5,"state":-1}
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  if(root != JsonObject::invalid() ) {
    root["v"] = 1;
    root["mac"] = WiFi.macAddress();
    root["co2"] = m_in.co2_ppm;
    root["temp"]= m_in.temperature;
    root["state"]= m_in.state;
    root.printTo(data);
    Serial.println(data.c_str());
  } else {
    Serial.println("Could not allocate jsonBuffer.");
  }
}

void publishMeasurements(const measurement_t &m)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    std::string data;
    prepareSchema(m, data);

    WiFiClient client; // TCP
    if (!client.connect(host, port))
    {
      Serial.println("Connection failed.");
      delay(1000);
    } else {
      if (client.connected())
      {
        client.write(data.c_str(), data.length());
        client.flush(1000); // wait maximum 1s
        client.stop();
      }
    }
  }
  else
  {
    setup_wifi();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting setup...");
  setup_wifi();
  co2_sensor_setup();
  Serial.println("...done!");
}

void loop()
{
  measurement_t m = mhz19_uart->getMeasurement();
  if (m.state < 0)
  {
    Serial.print("state: ");
    Serial.println(m.state);
  } else {
    publishMeasurements(m);
  }
  delay(1000);
}