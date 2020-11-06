#include <Arduino.h>
#include <WiFi.h>
#include "SSD1306Wire.h"
#include "MHZ19.h"

#include <ArduinoJson.h>

#include "wifi_credentials.h"   // const char* ssid = "ssid"; const char* password = "password";
#include "server_credentials.h" // const char* host = "google.com"; const uint16_t port = 80;


const uint8_t I2C_ADDRESS_DISPLAY = 0x3c;
const uint8_t _SDA = 0x05;
const uint8_t _SCL = 0x04;

SSD1306Wire display(I2C_ADDRESS_DISPLAY, _SDA, _SCL);

const int rx_pin = 13; //Serial rx pin no D2, green
const int tx_pin = 12; //Serial tx pin no D1, blue
MHZ19 *mhz19_uart = new MHZ19(rx_pin, tx_pin);

void wifi_setup()
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

void display_setup()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Waking up!");
  display.display();
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
  // example ~ 68 byte => {"v":1,"mac":"00:00:00:00:00:00","co2":2000,"temp":10,"state":-1}
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

void displayMeasurements(const measurement_t &m)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "CO2 [ppm]");
  display.drawString(72, 0, "Temp [deg]");
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 15, String(m.co2_ppm));
  display.drawString(84, 15, String(m.temperature));
  display.display();
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
        client.flush(); // wait maximum 1s
        client.stop();
      }
    }
  }
  else
  {
    wifi_setup();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting setup...");

  Serial.println("  display...");
  display_setup();
  Serial.println("  wifi...");
  wifi_setup();
  Serial.println("  co2 sensor...");
  co2_sensor_setup();

  Serial.println("...done!");
}

void loop()
{
  measurement_t m = mhz19_uart->getMeasurement();
  if(m.state != -1) {
    displayMeasurements(m);
    publishMeasurements(m);
  }
  delay(1000);
}