#include <Arduino.h>
#include <ESP8266WiFi.h>
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
  }
}
typedef struct snuffi_co2_v1
{
  uint8_t schema_version = 1;
  uint8_t mac_address[6] = {};
  uint16_t co2_ppm = 0;
  uint16_t temperature = 0;
  int8_t state = -1;
} snuffi_co2_v1_t;

void prepareSchema(const measurement_t &m_in, snuffi_co2_v1_t &packet)
{
  packet.schema_version = 1;
  WiFi.macAddress((uint8_t *)&(packet.mac_address));
  packet.co2_ppm = m_in.co2_ppm;
  packet.temperature = m_in.temperature;
  packet.state = m_in.state;
}

void publishMeasurements(const measurement_t &m)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClient client; // TCP
    if (!client.connect(host, port))
    {
      Serial.println("Connection failed.");
      delay(1000);
    } else {
      if (client.connected())
      {
        snuffi_co2_v1_t packet = {};
        prepareSchema(m, packet);
        client.write((uint8_t *)(&packet), sizeof(snuffi_co2_v1_t));
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
    Serial.print(m.state);
  }
  else
  {
    Serial.print("co2: ");
    Serial.print(m.co2_ppm);
    Serial.print("temp: ");
    Serial.print(m.temperature);
    Serial.print("state: ");
    Serial.print(m.state);
    publishMeasurements(m);
    Serial.println(" published!");
  }
  delay(1000);
}