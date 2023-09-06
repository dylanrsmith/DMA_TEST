#include <Arduino.h>
#include <SPI.h>
#include <ESP32DMASPISlave.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include "FS.h" // for SPIFFS
#include "SPIFFS.h"
#include "driver/mcpwm.h"
#include "esp32-hal.h"
#include "soc/soc_caps.h"
#include "driver/ledc.h"
#include "PWM.hpp"

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800); // on pin8
ESP32DMASPI::Slave slave;

int IO_0 = 0;          // IO0  = IO_0 - MAIN IO PIN - OUTPUT
int RGB = 10;          // IO10 = RGB
int BYPASS_RELAY = 18; // IO18 = Bypass relay
int digout = 19;       // IO19 = digout pin - Full Loop Bit flipping, set high at start of loop and low at end OUTPUT
PWM SYNC_PIN(9);       //  int SYNC_PIN = 9;

// Save data to file in json format
const char *filename = "/config.txt";
struct Config
{
  int boardType;
};
Config config;

float dutyCycle = 0.0;
int input1024 = 0;

int one = 0;
int a0 = 0;

static const uint32_t BUFFER_SIZE = 8;
static const uint32_t BUFFER_SIZE_C3 = 4;
uint8_t *spi_slave_tx_buf;
uint8_t *spi_slave_rx_buf;

const int ESP_D0 = 1;
const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D3 = 6;
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// freq out settings
const int pwmFreq = 2000;     // needs to work at 0
const int pwmResolution = 10; // 3;
// const int IO_0 = 0;
bool low_speed_freq = false;
int pwmState = LOW;
unsigned long time_now = 0;
unsigned long previousMillis = 0;
unsigned long previousMillisPWM = 0;

void loadConfiguration(const char *filename, Config &config)
{
  // Serial.println(F("checking config file..."));

  if (SPIFFS.begin(true))
  {
    File file = SPIFFS.open(filename, "r");

    StaticJsonDocument<3000> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
      // Serial.println(F("Failed to read file"));
    }

    config.boardType = doc["boardType"] | 0;
  }
  else
  {
    // Serial.println(F("SPIFFS fault"));
  }
}

void saveConfiguration(const char *filename, const Config &config)
{
  // Serial.println("");
  // Serial.println(F("saving.."));
  if (SPIFFS.begin(true))
  {
    File file = SPIFFS.open(filename, "w");
    if (!file)
    {
      // Serial.println(F("Failed to create file"));
      return;
    }
    StaticJsonDocument<2000> doc;

    // Set the values in the document
    doc["boardType"] = config.boardType;

    // Serialize JSON to file

    if (serializeJson(doc, file) == 0)
    {
      // Serial.println(F("Failed to write to file"));
    }

    // Close the file
    file.close();
  }
}

bool ValidateChecksum(uint8_t *buf)
{
  // if (sizeof(buf) == BUFFER_SIZE_SPI)
  {
    uint16_t checksum = 0;
    for (int i = 0; i < BUFFER_SIZE - 2; i++)
    {
      checksum += buf[i];
    }
    if ((checksum >> 8) == buf[BUFFER_SIZE - 2] && (checksum & 0xFF) == buf[BUFFER_SIZE - 1])
    {
      return true;
    }
  }

  return false;
}

// call free(XX) o the returned value to free malloc memory
uint8_t *AddChecksum(uint8_t *buf, uint8_t type)
{
  uint8_t *ret_buf = (uint8_t *)malloc(BUFFER_SIZE);
  if (sizeof(buf) == BUFFER_SIZE_C3)
  {
    uint16_t checksum = type;
    for (int i = 0; i < BUFFER_SIZE_C3; i++)
    {
      checksum += buf[i];
      ret_buf[i + 1] = buf[i];
    }

    ret_buf[0] = type;
    ret_buf[BUFFER_SIZE - 3] = 0x00;
    ret_buf[BUFFER_SIZE - 2] = checksum >> 8;
    ret_buf[BUFFER_SIZE - 1] = checksum & 0xFF;
  }
  return ret_buf;
}

void set_buffer()
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    spi_slave_tx_buf[i] = i & 0xFF;
  }
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

void setup()
{
  // Serial.begin(115200);
  delay(2000);
  RGBled.begin();
  RGBled.setBrightness(128);
  slave.setDataMode(SPI_MODE0);
  pinMode(ESP_D1, INPUT);              // sck io16b
  pinMode(ESP_D2, INPUT);              // mosi io16a
  pinMode(ESP_D4, INPUT);              // ss io13a
  pinMode(ESP_D5, OUTPUT);             // miso io13b
  loadConfiguration(filename, config); // Load JSON Type Bit

  // PWM input
  // SYNC_PIN.begin(true);
  pinMode(digout, OUTPUT);
  // to use DMA buffer, use these methods to allocate buffer
  spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  set_buffer();
  slave.setDataMode(SPI_MODE0);
  slave.setMaxTransferSize(BUFFER_SIZE);
  slave.begin(HSPI, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
  pinMode(0, INPUT);

  RGBled.setPixelColor(0, RGBled.Color(0, 50, 25)); // Moderately bright green color.
  RGBled.show();
}

void loop()
{
  digitalWrite(digout, HIGH);
  if (slave.remained() == 0)
  {
    slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  }

  bool valid = ValidateChecksum(spi_slave_rx_buf);

  if (valid)
  {
    while (slave.available()) //[$,1,2,3,\n]
    {
      // RGBled.setPixelColor(0, RGBled.Color(spi_slave_rx_buf[0], spi_slave_rx_buf[1], spi_slave_rx_buf[2])); // Moderately bright green color.
      // RGBled.show();

      uint32_t slot_val = 123456;
      uint8_t *slot1 = slave.allocDMABuffer(BUFFER_SIZE_C3);
      slot1[0] = (slot_val >> 24) & 0xFF;
      slot1[1] = (slot_val >> 16) & 0xFF;
      slot1[2] = (slot_val >> 8) & 0xFF;
      slot1[3] = slot_val & 0xFF;

      // for (int i = 0; i<4; i++){
      //     Serial.printf("%d ",slot1[i]);
      // }
      // Serial.println();

      uint8_t *bufffff = AddChecksum(slot1, spi_slave_rx_buf[0]);
      spi_slave_tx_buf = bufffff;
      free(bufffff);

      // for (int i = 0; i<BUFFER_SIZE; i++){
      //     Serial.printf("%d ",spi_slave_tx_buf[i]);
      // }
      // Serial.println();

      slave.pop();
    }
  }

  digitalWrite(digout, LOW);
}