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
int a0 =0;

static const uint32_t BUFFER_SIZE = 8;
uint8_t *spi_slave_tx_buf;
uint8_t *spi_slave_rx_buf;

const int ESP_D0 = 1;
const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D3 = 6;
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// freq out settings
const int pwmFreq = 10;       // needs to work at 0
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

void low_freq(const int pin, const int freq)
{
  unsigned long currentMillisPWM = millis();

  // 500 / freq???
  float period = 1000.0 / freq; // 1000 since we are using milliseconds to keep track
  // float period = 1.0/freq;
  // Serial.printf("period: %f\n",period);
  if (currentMillisPWM - previousMillisPWM >= period)
  {
    // save the last time you blinked the LED
    previousMillisPWM = currentMillisPWM;

    // if the LED is off turn it on and vice-versa:
    if (pwmState == LOW && freq != 0)
    {
      pwmState = HIGH;
    }
    else
    {
      pwmState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(pin, pwmState);
  }
}

void set_buffer()
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    spi_slave_tx_buf[i] = (0xFF - i) & 0xFF;
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
  pinMode(ESP_D1, INPUT);  // sck io16b
  pinMode(ESP_D2, INPUT);  // mosi io16a
  pinMode(ESP_D4, INPUT);  // ss io13a
  pinMode(ESP_D5, OUTPUT); // miso io13b
  // Serial.println("starting..");

  // Load JSON Type Bit
  loadConfiguration(filename, config);
  // Serial.printf("The board type is: %d\n", config.boardType);

  // if (pwmFreq > 200)
  // {
  //   // freq out setup
  //   ledcSetup(1, pwmFreq, 10); // channel,freq,pwm  FOR PWM ONLY
  //   ledcAttachPin(IO_0, 1);    // pin, channel
  //   ledcWriteTone(1, pwmFreq); // 20000
  //   ledcWrite(1, 650);         // channel, duty value 0..255;
  //   low_speed_freq = false;
  // }
  // else
  // {
  //   // BitBang
  //   pinMode(IO_0, OUTPUT);
  //   low_speed_freq = true;
  // }
  // // PWM input
  // SYNC_PIN.begin(true);

  pinMode(digout, OUTPUT);

  // to use DMA buffer, use these methods to allocate buffer
  spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  // set_buffer();
  // delay(5000);
  // slave device configuration
  slave.setDataMode(SPI_MODE0);
  slave.setMaxTransferSize(BUFFER_SIZE);
  slave.begin(HSPI, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS

  // Serial.println("Setup Complete");
  spi_slave_tx_buf[0] = 5;
  spi_slave_tx_buf[1] = 5;
  spi_slave_tx_buf[2] = 5;
  spi_slave_tx_buf[3] = 5;
  spi_slave_tx_buf[4] = 5;
  spi_slave_tx_buf[5] = 5;
  spi_slave_tx_buf[6] = 5;
  spi_slave_tx_buf[7] = 5;

  pinMode(0,INPUT);
}

void loop()
{
  digitalWrite(digout, HIGH);
  // if (slave.remained() == 0)
  // {
  //   slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  // }

  // while (slave.available()) //[$,1,2,3,\n]
  // {
  //   // RGBled.setPixelColor(0, RGBled.Color(spi_slave_rx_buf[0], spi_slave_rx_buf[1], spi_slave_rx_buf[2])); // Moderately bright green color.
  //   // RGBled.show();
  //   // for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  //   // {
  //   //   //spi_slave_tx_buf[i] = (0xFF - (uint8_t(spi_slave_rx_buf[i]))) & 0xFF;
  //   //   spi_slave_tx_buf[i] = spi_slave_rx_buf[i] + 1;
  //   // }

  //   // if(spi_slave_rx_buf[4] == 255)
  //   // {
  //   //   spi_slave_rx_buf[4] = 0;
  //   // }
  //   spi_slave_tx_buf[4] = spi_slave_rx_buf[4] + 1;
  //   slave.pop();
  // }

  a0 = analogRead(0);

  // if (one == 0)
  // {
  //   ledcWrite(1, 128);
  //   one = 2;
  // }

  digitalWrite(digout, LOW);
  // unsigned long currentMillis = millis();
  // if (!low_speed_freq)
  // {
  //   if (currentMillis - previousMillis >= 500)
  //   {
  //     previousMillis = currentMillis;

  //     Serial.print("microseconds active: ");
  //     Serial.println(SYNC_PIN.getValue());

  //     input1024 = input1024 + 2;
  //     ledcWrite(1, input1024 * 4);

  //     // dutyCycle = ((input1024 *4)/1024)*100;
  //     dutyCycle = (static_cast<float>(input1024 * 4) / 1024.0) * 100.0;

  //     // Serial.printf("Writing duty cycle of : %f\n ", dutyCycle);
  //     // Serial.printf("(actualinput:%d)\n", (input1024 * 4));
  //     if (input1024 > 250) // start over if we reach max input
  //     {
  //       input1024 = 0;
  //     }
  //   }
  // }
  // else if (low_speed_freq)
  // {
  //   if (currentMillis - previousMillis >= 500)
  //   {
  //     previousMillis = currentMillis;
  //     // Serial.print("microseconds active: ");
  //     // Serial.println(SYNC_PIN.getValue());
  //   }
  //   low_freq(IO_0, pwmFreq); // Bit bang a lower freq pwm signal
  // }
}