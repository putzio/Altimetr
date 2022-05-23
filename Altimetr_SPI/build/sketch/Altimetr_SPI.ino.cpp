#line 1 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
#include <Arduino.h>
#include <SPI.h>
#include <avr/eeprom.h>
#include <Adafruit_BMP280.h>
#include <SerialFlash.h>

// pinout
// CS -> write LOW to choose the salve
#define WRITE_PROTECTION 0  // WP - PA4
#define CS_FLASH 1          // PA5
#define SEND_DATA_UART_EN 3 // INT - PA7
#define RX 4                // PB3
#define TX 5                // PB2
#define LED 6               // PB1
#define CS_BMP 7            // PB0
#define UPDI 11             // PA0
#define MOSI 8              // PA1
#define MISO 9              // PA2
#define SCK 10              // PA3

#define seaLevelhPa 1020.0
#define TIME_INTERVAL 1000 // ms

SerialFlashFile flash;
Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);

float maxHightEEM EEMEM;
float maxHightRAM;
float initialHight;
uint64_t t = 0; // timer updated with millis()
#line 31 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
void setup();
#line 31 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
void setup()
{
  // put your setup code here, to run once:
  for (int i = 0; i < 11; i++) // For all pins
    pinMode(i, OUTPUT);
  pinMode(SEND_DATA_UART_EN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("START");

  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }
  /*
  Mode:           Normal -> 0xF4 [1:0] -> 11
  Oversampling:   Standard Resolution
  osrs_p:         x4 -> 0xF4 [4:2] -> 011
  osrs_t:         x1 -> 0xF4 [7:5] -> 001
  ODR[Hz]:        7.3
  RMS Noise[cm]:  6.4
  3 wire SPI:     false -> 0xF5[0] -> 0
  IIR filter:     4 -> 0xF5[4:2] -> 010?
  standby [ms]:   125 -> 0xF5 [7:5] -> 010
  */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

  initialHight = bmp.readAltitude(seaLevelhPa);
  if (Serial.available())
  {
    Serial.print("INITIAL HIGHT:\t");
    Serial.println(initialHight);
  }

  // Flash
  if (!SerialFlash.begin(CS_FLASH))
  {
    while (1)
    {
      Serial.println("Unable to access SPI Flash chip");
      delay(1000);
    }
  }
  unsigned char id[5];
  SerialFlash.readID(id);
  unsigned long size = SerialFlash.capacity(id);

  if (size > 0 && Serial.available())
  {
    Serial.print("Flash Memory has ");
    Serial.print(size);
    Serial.println(" bytes.");
  }

  void loop()
  {
    if (millis() - t > TIME_INTERVAL)
    {
      t = millis();
      float hight = bmp.readAltitude(seaLevelhPa) - initialHight;
      if (hight > maxHightRAM)
      {
        maxHightRAM = hight;
      }
      // If the rocket has already reached the highest point write the max hight to flash
      else
      {
        eeprom_update_float(&maxHightEEM, maxHightRAM);
      }
      if (Serial.available())
      {
        Serial.print("Hight:\t");
        Serial.println(hight);
      }

      if (Serial.available() && !digitalRead(SEND_DATA_UART_EN))
      {
        float readHight = eeprom_read_float(&maxHightEEM);
        Serial.print("Max Hight:\t");
        Serial.println(readHight);
      }
    }
  }

