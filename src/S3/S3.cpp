/*
  CONN/!DAT
  D0i D1i D2i D3i D4i D5o
  1   0   0   0   0   x  SMART IO CONNECT OE=0
  1   0   0   1   0   x   SMART IO CONNECT OE=1
  1   1   0   0   0   x   SIMPLE IO CONNECT OE=0
  1   1   0   1   0   x   SIMPLE IO CONNECT OE=1
  1   0   1   0   0   x   RELAY IO CONNECT

  D0i D1i D2i D3i   D4i     D5o
  0   SCK MO  SYNC  LATCH   MISO    SMART IO CONNECT DATA MODE
  0   SCK MO  PL    LATCH   MISO  SIMPLE IO CONNECT DATA MODE
  0   SCK MO  OE    LATCH   x RELAY IO CONNECT DATA MODE
*/

#include "EasyCAT.h"
// #include <SPI.h>
#include "Wire.h"
#include "XPowersLib.h" //https://github.com/lewisxhe/XPowersLib
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE = 4;

uint8_t *spi_master_tx_buf;
uint8_t *spi_master_rx_buf;

XPowersAXP2101 PMU;

// FPGA PINS
#define PIN_BTN 0
#define PIN_IIC_SDA 38
#define PIN_IIC_SCL 39
#define PIN_PMU_IRQ 40
// vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
#define PIN_LED 46
// #define CLICK_RST 9

const int ESP_D0 = 1;
const int ESP_D1 = 3;
const int ESP_D2 = 5;
const int ESP_D3 = 6;
const int ESP_D4 = 4;
const int ESP_D5 = 2;

const char SpiCS_Pin = 11; // ethercat click ss

int color = 0;
bool blink = true;
const int OUTPUTS_COUNT = 8;
const long interval = 200;
unsigned long previousMs = 0;
unsigned long previousMillis = 0;

// non DMA
// static const int spiClk = 20000000;
// SPIClass *vspi = NULL;
// SPIClass * hspi = NULL;

float factor1, factor2;
int ind = 0;
byte testData = 0;
byte greenColor = 255;
byte blueColor = 255;
byte redColor = 255;
int magicNumber = 0;
bool pdo_updated = true;
bool rainbow = false;

int count = 0;

void setOUTPUT()
{
    pinMode(ESP_D0, OUTPUT);
    pinMode(ESP_D1, OUTPUT); // sck
    pinMode(ESP_D2, OUTPUT); // mosi
    pinMode(ESP_D3, OUTPUT);
    pinMode(ESP_D4, OUTPUT); // sss
}

void Application()
{
    int prev_num = magicNumber;
    magicNumber = EasyCAT_BufferOut.Cust.Output2;

    if (prev_num != magicNumber)
    {
        pdo_updated = true;
        Serial.println(magicNumber);
    }
    else
    {
        pdo_updated = false;
    }

    switch (magicNumber)
    {
    case 1: // blue
        redColor = 0;
        greenColor = 0;
        blueColor = 255;
        break;
    case 2: // green
        redColor = 0;
        greenColor = 255;
        blueColor = 0;
        break;
    case 3: // red
        redColor = 255;
        greenColor = 0;
        blueColor = 0;
        break;
    case 4: // orange
        redColor = 230;
        greenColor = 140;
        blueColor = 14;
        break;
    case 5: // pink?
        redColor = 255;
        greenColor = 78;
        blueColor = 192;
        break;
    case 6:
        rainbow = true;
        break;
    case 7:
        rainbow = false;
        break;
    }

    // write to slave TxPDO
    // dummy value, so that value will be seen constantly changing by ecat master
    uint32_t actualPosition = EasyCAT_BufferIn.Cust.Input1 + 1;
    EasyCAT_BufferIn.Cust.Input1 = actualPosition;
}

void digital_macro(bool one, bool two, bool three, bool four, bool zero)
{
    digitalWrite(ESP_D1, one);
    digitalWrite(ESP_D2, two);
    digitalWrite(ESP_D3, three);
    digitalWrite(ESP_D4, four);
    digitalWrite(ESP_D0, zero);
}

void shift_macro()
{
    // uint8_t buf[] = {'$', redColor, greenColor, blueColor, '\n'};
    // vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
    // pinMode(vspi->pinSS(), OUTPUT);              // VSPI SS
    // vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    // digitalWrite(vspi->pinSS(), LOW);
    // vspi->transfer(&buf, 5); // other way to do it (slower)
    // digitalWrite(vspi->pinSS(), HIGH);
    // vspi->endTransaction();
    // vspi->end();
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);
}

void rainbowAnimation()
{
    int stepSize = 1023;
    int dividerVar = 341;
    int subVar = 682;
    switch ((int)((ind % stepSize) / dividerVar))
    {
    case 0:
        factor1 = 1.0 - ((float)(ind % stepSize - 0 * dividerVar) / dividerVar);
        factor2 = (float)((int)(ind - 0) % stepSize) / dividerVar;
        // strip_0.strip.setPixelColor(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
        redColor = int(255 * factor1 + 0 * factor2);
        greenColor = int(0 * factor1 + 255 * factor2);
        blueColor = int(0 * factor1 + 0 * factor2);
        break;
    case 1:
        factor1 = 1.0 - ((float)(ind % stepSize - 1 * dividerVar) / dividerVar);
        factor2 = (float)((int)(ind - dividerVar) % stepSize) / dividerVar;
        // strip_0.strip.setPixelColor(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
        redColor = int(0 * factor1 + 0 * factor2);
        greenColor = int(255 * factor1 + 0 * factor2);
        blueColor = int(0 * factor1 + 255 * factor2);
        break;
    case 2:
        factor1 = 1.0 - ((float)(ind % stepSize - 2 * dividerVar) / dividerVar);
        factor2 = (float)((int)(ind - subVar) % stepSize) / dividerVar;
        // strip_0.strip.setPixelColor(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
        redColor = int(0 * factor1 + 255 * factor2);
        greenColor = int(0 * factor1 + 0 * factor2);
        blueColor = int(255 * factor1 + 0 * factor2);
        break;
    }
    if (ind >= stepSize)
    {
        ind = 0;
    }
    else
        ind++;
}

void set_buffer()
{
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        spi_master_tx_buf[i] = i & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE);
}

void setup()
{
    Serial.begin(115200);
    delay(5000);
    Serial.println("SERIAL OPENED");

    // EtherCAT SETUP
    // pinMode(40, OUTPUT); // EC RST pin
    // digitalWrite(40, HIGH);
    // delay(500);
    // Serial.print("ETHER CAT Initialization...");
    // EasyCAT_Init(SpiCS_Pin, ASYNC);

    Serial.println("SERIAL OPENED");
    // FPGA SETUP
    bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_IIC_SDA, PIN_IIC_SCL);
    if (result == false)
    {
        Serial.println("PMU is not online...");
        while (1)
            Serial.println("Error PMU");
        delay(500);
    }

    // FPGA Power Supply
    PMU.setDC4Voltage(1200);   // Here is the FPGA core voltage. Careful review of the manual is required before modification.
    PMU.setALDO1Voltage(3300); // BANK0 area voltage
    PMU.setALDO2Voltage(3300); // BANK1 area voltage
    PMU.setALDO3Voltage(3300); // BANK2 area voltage
    PMU.setALDO4Voltage(3300); // BANK3 area voltage
    PMU.enableALDO1();
    PMU.enableALDO2();
    PMU.enableALDO3();
    PMU.enableALDO4();
    PMU.disableTSPinMeasure();
    delay(1000);

    Serial.println("Starting...PMU SETUP ");
    // Blue LED
    pinMode(PIN_LED, OUTPUT); // GPIO46
    digitalWrite(PIN_LED, HIGH);
    // delay(5000);

    // put FPGA in smart mode with OE Enabled
    pinMode(ESP_D1, OUTPUT); // CLK
    pinMode(ESP_D2, OUTPUT); // MOSI
    pinMode(ESP_D3, OUTPUT); // SYNC
    pinMode(ESP_D4, OUTPUT); // CS
    pinMode(ESP_D5, INPUT);
    pinMode(ESP_D0, OUTPUT); // MODE

    // smart mode with OE enabled:
    digitalWrite(ESP_D0, LOW);
    digitalWrite(ESP_D1, LOW);
    digitalWrite(ESP_D2, LOW);
    digitalWrite(ESP_D3, LOW);
    digitalWrite(ESP_D4, LOW); // going to use D4 for SS for demo - CHANGE THIS TO LATCH FOR REAL CODE

    digitalWrite(ESP_D0, HIGH); // all is applied on rising edge
    digitalWrite(ESP_D0, LOW);
    delay(10);
    // digitalWrite(ESP_D1, LOW);
    // digitalWrite(ESP_D2, LOW);
    // digitalWrite(ESP_D3, LOW);
    // digitalWrite(ESP_D4, LOW);
    // delay(10);
    delay(10);

    // to use DMA buffer, use these methods to allocate buffer
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);
    // start DMA master
    // set_buffer();
    delay(1000);
    master.setDataMode(SPI_MODE0);          // default: SPI_MODE0
    master.setFrequency(2000000);           // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE); // default: 4092 bytes

    // try different spi buses? vsp,hspi,fspi...
    master.begin(HSPI, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // default: HSPI (CLK MISO MOSI CS)

    Serial.println("Starting...SETUP COMPLETE");

    spi_master_tx_buf[0] = uint8_t(1);
    spi_master_tx_buf[1] = uint8_t(2);
    // spi_master_tx_buf[2] = uint8_t(3);
    // spi_master_tx_buf[3] = uint8_t(4);
}

void loop()
{    
    spi_master_tx_buf[0] = count++;    
    Serial.println("LOOP");
    digitalWrite(ESP_D3, HIGH); // OE
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);
    digitalWrite(ESP_D3, LOW);
    delay(1000);

    for (size_t i = 0; i < BUFFER_SIZE; ++i)
    {
      Serial.printf("Master rx: %d\n", spi_master_rx_buf[i]);      
    }

}