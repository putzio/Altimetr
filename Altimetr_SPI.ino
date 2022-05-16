//#include <Arduino.h>
//#include <SPI.h>
#include <avr/eeprom.h>
#include <Adafruit_BMP280.h>
#include <SPIMemory.h>
#include <SPI.h>
#include <avr/io.h>

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

#define SEA_LEVEL_HPA 1020.0
#define TIME_INTERVAL 1000 // ms
#define SEA_LEVEL_ADRESS 0x00 
#define START_ADRESS SEA_LEVEL_ADRESS

#define F_SPI 1000000 // 1 MHz -> the same frequency is required, so we take the lower one from BMP and set it to FLASH

SPIFlash *flash;
Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);

float initialHight;
uint64_t t = 0; // timer updated with millis()
bool flashOn = false;
uint32_t addr = 0x00;

float seaLevel;

void FlashBegin(bool *flashState = &flashOn)
{
  if (*flashState)
    Serial.println("Flash is already on");
  else
  {
    flash = new SPIFlash(CS_FLASH);
    flash->begin();
    *flashState = true;
  }
}
void FlashEnd(bool *flashState = &flashOn)
{
  if (*flashState)
  {
    SPI.end();
    delete flash;
    *flashState = false;
  }
  else
  {
    Serial.println("No flash object was used");
  }
}
// checks if the adress is empty
bool CheckAdress(uint32_t addr, bool flashState = flashOn)
{
  if (!flashState)
  {
    FlashBegin();
    if (Serial.available())
    {
      Serial.println("Initialised Flash in CheckAdress() function");
    }
  }
  int32_t val = flash->readLong(addr);
  if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
  {
    FlashEnd();
  }
  return (val == -1);
}

void FindEmptyAdress(volatile uint32_t *startingAdress, bool flashState = flashOn)
{
  if (!flashState)
  {
    FlashBegin();
    if (Serial.available())
    {
      Serial.println("Initialised Flash in FindEmptyAdress() function");
    }
  }

  uint32_t size = flash->getCapacity();
  while (1)
  {
    if (*startingAdress > size)
    {
      *startingAdress = 0;
      if (Serial.available())
      {
        Serial.println("FULL MEMORY");
      }
      break;
    }
    if (CheckAdress(*startingAdress))
      break;
    *startingAdress += 8;
  }
  if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
  {
    FlashEnd();
  }
}

void UartInit(uint16_t baud)
{
  Serial.begin(baud);
  while (!Serial.available() && !digitalRead(SEND_DATA_UART_EN))
    Serial.println("Waiting for connection...");
  if(Serial.available())
    Serial.println("Device connected");
}

void BMP280Init()
{
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
}

void setup()
{
  //Set all pins as OUTPUT:
  for (int i = 0; i < 11; i++) 
    pinMode(i, OUTPUT);
  
  pinMode(SEND_DATA_UART_EN, INPUT_PULLUP);
  UartInit(115200);
  FlashBegin();
  seaLevel = flash->readFloat(SEA_LEVEL_ADRESS);
  if(seaLevel<100.0)
    seaLevel = SEA_LEVEL_HPA; 
  FlashEnd();

  BMP280Init();  
  initialHight = bmp.readAltitude(seaLevel);
  if (Serial.available())
  {
    Serial.print("INITIAL HIGHT:\t");
    Serial.println(initialHight);
    Serial.print("TEMP:\t");
    Serial.println(bmp.readTemperature());
    Serial.print("PRESSURE:\t");
    Serial.println(bmp.readPressure());
  }

  // Flash
  FlashBegin();
  if (Serial.available())
  {
    Serial.println(flash->readLong(0x00));
  }
  FlashEnd();
  // if (Serial.available())
  // {
  //   Serial.print("Read Flash:\t");
  //   Serial.println(flash.readFloat(0x00));
  //   unsigned long size = flash.getCapacity();
  //   Serial.print("Flash Memory has ");
  //   Serial.p/rint(size);
  //   Serial.println(" bytes.");
  // }

  if (digitalRead(SEND_DATA_UART_EN))
  {
    // flash.eraseChip();
  }

  volatile uint32_t start;
  FindEmptyAdress(&start);
  if(Serial.available())
  {
    Serial.print("START ADDR:\t");
    Serial.println(start);
    Serial.print("How many measurements are stored:\t");
    Serial.println(start / 8);
  }
}

void loop()
{
  if (millis() - t > TIME_INTERVAL)
  {
    t = millis();

    if (!digitalRead(SEND_DATA_UART_EN))
    {
      addr = 0x04;
      FlashBegin();
      String recivedData = Serial.readString();
      if(recivedData != "-1")
        {
          if(recivedData.substring(0,3) =="#P:")
          {
            //Change Preassure
            float preassure =  recivedData.substring(3,recivedData.indexOf("&")).toFloat();
            flash->writeFloat(SEA_LEVEL_ADRESS, preassure);
          }
          if(recivedData.substring(0,3) =="###")
          {
            //Clear Flash
            flash->eraseChip();
          }
        }
      
      uint32_t flashCapacity = flash->getCapacity();
      uint32_t flashTime = 0,newFlashTime = 0;
      while(addr<flashCapacity)
      {
        newFlashTime =  flash->readLong(addr);
        if(flashTime>newFlashTime)
          Serial.println("end");//the new series of measurements has started
        flashTime = newFlashTime;
        // Serial.print("ADDR:\t");
        // Serial.print(addr, HEX);
        Serial.print("\t#t:\t");
        Serial.print(newFlashTime);
        addr += 4;
        Serial.print("\t&h:\t");
        Serial.print(flash->readFloat(addr));
        Serial.println("$");
        addr += 4;        
      }      
      Serial.println("END");      
    }
    else 
    {
      float hight = bmp.readAltitude(seaLevel); // - initialHight;
      // FlashBegin();
      //  flash->writeLong(addr, millis());
      addr += 4;
      // flash->writeFloat(addr, hight);
      addr += 4;
      // FlashEnd();
    }
  }
}
/*
To do:

->MIKRO
Czyszczenie flasha na komendę z UARTa
Wpisywanie do pamięci ciśnienia przez UART
kilka pomiarów na jednej pamięci

->PC
Komunikacja -> suma kontrolna, arbitraż
Wykres -> dopasowanie osi
Wysyłanie komendy kasowania i ustawienia ciśnienia
Obliczanie ciśnienia na poziomie morza na podstawie aktualnej wysokości i wskazań z BMP280 
*/