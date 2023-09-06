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
#include <SPI.h>
#include "Wire.h"
#include "XPowersLib.h" //https://github.com/lewisxhe/XPowersLib
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE_SPI = 8; // For SPI Protocol from S3 and C3
static const uint32_t BUFFER_SIZE_EC = 4;  // For EtherCAT Protocol from Master
static const uint32_t BUFFER_SIZE_CS = 4;  // For Chip Select / Shift Register.

uint8_t *spi_master_tx_buf;
uint8_t *spi_master_rx_buf;

uint8_t *cs_master_tx_buf;
uint8_t *cs_master_rx_buf;

uint8_t **spi_master_tx_buf_multi;

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
const long delay_interval = 10;

float factor1, factor2;
int ind = 0;
byte testData = 0;
byte greenColor = 255;
byte blueColor = 255;
byte redColor = 255;
int magicNumber = 0;
bool pdo_updated = true;
bool rainbow = false;
uint32_t slot1_val = 0;
uint8_t type = 0;

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
    // Get output
    // int prev_num = magicNumber;
    // magicNumber = EasyCAT_BufferOut.Cust.Slot1;

    // write to slave TxPDO
    // dummy value, so that value will be seen constantly changing by ecat master
    uint32_t actualPosition = EasyCAT_BufferIn.Cust.Slot1 + 1;
    EasyCAT_BufferIn.Cust.Slot1 = actualPosition + 5;
}

void digital_macro(bool one, bool two, bool three, bool four, bool zero)
{
    digitalWrite(ESP_D1, one);
    digitalWrite(ESP_D2, two);
    digitalWrite(ESP_D3, three);
    digitalWrite(ESP_D4, four);
    digitalWrite(ESP_D0, zero);
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
    // Slot Board
    for (uint32_t i = 0; i < BUFFER_SIZE_SPI; i++)
    {
        spi_master_tx_buf[i] = i & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE_SPI);

    // Shift Register
    for (uint32_t i = 0; i < BUFFER_SIZE_CS; i++)
    {
        cs_master_tx_buf[i] = 0x00 & 0xFF;
    }
    memset(cs_master_rx_buf, 0, BUFFER_SIZE_CS);
}

bool ValidateChecksum(uint8_t *buf)
{
    // if (sizeof(buf) == BUFFER_SIZE_SPI)
    //  {
    uint16_t checksum = 0;
    for (int i = 0; i < BUFFER_SIZE_SPI - 2; i++)
    {
        checksum += buf[i];
    }
    if ((checksum >> 8) == buf[BUFFER_SIZE_SPI - 2] && (checksum & 0xFF) == buf[BUFFER_SIZE_SPI - 1])
    {
        return true;
    }
    // }

    return false;
}

// call free(XX) o the returned value to free malloc memory
uint8_t *AddChecksum(uint8_t *buf, uint8_t type)
{
    uint8_t *ret_buf = (uint8_t *)malloc(BUFFER_SIZE_SPI);
    if (sizeof(buf) == BUFFER_SIZE_EC)
    {
        uint16_t checksum = type;
        for (int i = 0; i < BUFFER_SIZE_EC; i++)
        {
            checksum += buf[i];
            ret_buf[i + 1] = buf[i];
        }

        ret_buf[0] = type;
        ret_buf[BUFFER_SIZE_SPI - 3] = 0x00;
        ret_buf[BUFFER_SIZE_SPI - 2] = checksum >> 8;
        ret_buf[BUFFER_SIZE_SPI - 1] = checksum & 0xFF;
    }
    return ret_buf;
}

void EtherCAT_Read()
{
    type = EasyCAT_BufferOut.Cust.type;
    // slot1_val = EasyCAT_BufferOut.Cust.Slot1;
    slot1_val = 25252;

    uint8_t *slot1 = master.allocDMABuffer(BUFFER_SIZE_EC);
    // Serial.printf("EtherCATREAD: type = %d\nslot1_val = %d\n",type,slot1_val);

    slot1[0] = (slot1_val >> 24) & 0xFF;
    slot1[1] = (slot1_val >> 16) & 0xFF;
    slot1[2] = (slot1_val >> 8) & 0xFF;
    slot1[3] = slot1_val & 0xFF;

    uint8_t *bufffff = AddChecksum(slot1, type);
    spi_master_tx_buf = bufffff;
    free(bufffff);
    // spi_master_tx_buf[7] = ValidateChecksum(spi_master_rx_buf);
    // spi_master_tx_buf[7] = ValidateChecksum(spi_master_tx_buf);
}

void EtherCAT_Write()
{
    // Convert the byte array into a uint32_t
    uint32_t result = 0;

    for (int i = 1; i < 5; i++)
    {
        // Shift the uint32_t left by 8 bits and OR it with the current uint8_t element
        result = (result << 8) | spi_master_rx_buf[i];
    }

    // Serial.printf("EtherCAT WRITE: result = %d\n",result);

    EasyCAT_BufferIn.Cust.Slot1 = result;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SERIAL OPENED");

    // EtherCAT SETUP
    pinMode(40, OUTPUT); // EC RST pin
    digitalWrite(40, HIGH);
    Serial.print("ETHER CAT Initialization...");
    EasyCAT_Init(SpiCS_Pin, ASYNC);

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

    // to use DMA buffer, use these methods to allocate buffer
    // Slot Boards
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE_SPI);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE_SPI);

    // Shift Registers
    cs_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE_CS);
    cs_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE_CS);

    delay(10);
    master.setDataMode(SPI_MODE0);              // default: SPI_MODE0
    master.setFrequency(20000000);              // 40/30 million seems to create errors
    master.setMaxTransferSize(BUFFER_SIZE_SPI); // default: 4092 bytes

    // try different spi buses? vsp,hspi,fspi...
    master.begin(HSPI, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // default: HSPI (CLK MISO MOSI CS)

    Serial.println("Starting...SETUP COMPLETE");

    set_buffer();

    cs_master_tx_buf[0] = uint8_t(5);
    cs_master_tx_buf[1] = uint8_t(6);
    cs_master_tx_buf[2] = uint8_t(7);
    cs_master_tx_buf[3] = uint8_t(8);

    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);
}

void loop()
{
    digitalWrite(7, HIGH);

    unsigned long currentMs = millis();

    // non blocking loop
    ////////////////////////////////
    if (currentMs - previousMs >= delay_interval)
    {
        previousMs = currentMs;

        // EtherCAT Functions
        // Application();
        //////////////////////////////////
        EtherCAT_Read();
        EasyCAT_MainTask();
        //////////////////////////////////

        // blink blue LED
        if (digitalRead(46) == 1)
        {
            digitalWrite(46, LOW);
        }
        else
        {
            digitalWrite(46, HIGH);
        }

        // SPI transaction
        ///////////////////////////////////
        for (int i = 0; i < 1; i++)
        {
            spi_master_tx_buf[0] = i;
            digitalWrite(ESP_D3, HIGH); // OE
            master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE_SPI);
            digitalWrite(ESP_D3, LOW);

            bool valid = ValidateChecksum(spi_master_rx_buf);
            Serial.println(valid);

            for (int i = 0; i < BUFFER_SIZE_SPI; i++)
            {
                Serial.printf("%d ", spi_master_rx_buf[i]);
            }
            Serial.println();

            // cs_master_tx_buf[0] = i;
            // master.transfer(cs_master_tx_buf, cs_master_rx_buf, BUFFER_SIZE_CS);
        }

        // for (int i = 0; i<BUFFER_SIZE_SPI; i++){
        //     Serial.printf("%d ",spi_master_tx_buf[i]);
        // }
        // Serial.println();

        EtherCAT_Write();
    }

    delay(2);

    digitalWrite(7, LOW);
}