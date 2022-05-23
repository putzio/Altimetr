//#include <avr/eeprom.h>
#include <Adafruit_BMP280.h>
#include <SPIMemory.h>
#include <SPI.h>
#include "lib/config.h"
//#include "lib/flashFunctions.h"

// // // pinout
// // // CS -> write LOW to choose the salve
// #define WRITE_PROTECTION 0  // WP - PA4
// #define CS_FLASH 1          // PA5
// #define SEND_DATA_UART_EN 3 // INT - PA7
// #define RX 4                // PB3
// #define TX 5                // PB2
// #define LED 6               // PB1
// #define CS_BMP 7            // PB0
// #define UPDI 11             // PA0
// #define MOSI 8              // PA1
// #define MISO 9              // PA2
// #define SCK 10              // PA3

// #define EEPROM_BYTES 256

// #define SEA_LEVEL_HPA 1020
// #define TIME_INTERVAL 1000 // ms
// #define START_ADRESS 0x00

// #define F_SPI 1000000 // 1 MHz -> the same frequency is required, so we take the lower one from BMP and set it to FLASH
void FlashBegin(SPIFlash *flash, bool *flashState)
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

void FlashEnd(SPIFlash *flash, bool *flashState)
{
//  if (*flashState)
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
bool CheckAdress(SPIFlash *flash, uint32_t addr, bool flashState)
{
 if (!flashState)
 {
   FlashBegin(flash);
   if (Serial.available())
   {
     Serial.println("Initialised Flash in CheckAdress() function");
   }
 }
  int32_t val = flash->readLong(addr);
 if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
 {
   FlashEnd(flash);
 }
  return (val == -1);
}

void FindEmptyAdress(SPIFlash *flash, volatile uint32_t *startingAdress, bool flashState)
{
 if (!flashState)
 {
   FlashBegin(flash);
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
    if (CheckAdress(flash, *startingAdress))
      break;
    *startingAdress += 8;
  }
 if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
 {
   FlashEnd(flash);
 }
}


float initialHight;
uint32_t t = 0; // timer updated with millis()
bool flashOn = false;
uint32_t addr = START_ADRESS;

uint8_t seaLevelPreassureAdress = 0x00;

// void FlashBegin(SPIFlash *flash, bool *flashState = &flashOn);
// void FlashEnd(SPIFlash *flash, bool *flashState = &flashOn);
// // checks if the adress is empty
// bool CheckAdress(SPIFlash *flash, uint32_t addr, bool flashState = flashOn);
// //find the first em
// void FindEmptyAdress(SPIFlash *flash, volatile uint32_t *startingAdress, bool flashState = flashOn);

void UartInit(uint16_t baud);

void BMP280Init(Adafruit_BMP280 *bmp);

// uint8_t FindLastAdressEEM();

//void UpdatePreassure(uint16_t newPreassure);
//
//uint16_t ReadSavedPreassure();

void setup()
{
  //Set all pins as OUTPUT:
  for (int i = 0; i < 11; i++) 
    pinMode(i, OUTPUT);
  //and one pin as an input to know, when PC is connected
  pinMode(SEND_DATA_UART_EN, INPUT_PULLUP);
  UartInit(115200);
  while (!Serial && !digitalRead(SEND_DATA_UART_EN));
  if(Serial.available())
    Serial.println("Device connected");

//?
//  {
//    Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
//    BMP280Init(&bmp);  
////    initialHight = bmp.readAltitude(ReadSavedPreassure());
//    if (Serial.available())
//    {
//      Serial.print("INITIAL HIGHT:\t");
//      Serial.println(initialHight);
//      Serial.print("TEMP:\t");
//      Serial.println(bmp.readTemperature());
//      Serial.print("PRESSURE:\t");
//      Serial.println(bmp.readPressure());
//    }
//  }

  // Flash
//  SPIFlash *flash;
//  FlashBegin(flash,flashOn);
//  if (Serial)
//  {
//    Serial.println(flash->readLong(0x00));
//  }
//  FlashEnd(flash);
  // if (Serial.available())
  // {
  //   Serial.print("Read Flash:\t");
  //   Serial.println(flash.readFloat(0x00));
  //   unsigned long size = flash.getCapacity();
  //   Serial.print("Flash Memory has ");
  //   Serial.p/rint(size);
  //   Serial.println(" bytes.");
  // }

//  volatile uint32_t start;
//  FindEmptyAdress(flash, &start,flashOn);
//  if(Serial.available())
//  {
//    Serial.print("START ADDR:\t");
//    Serial.println(start);
//    Serial.print("How many measurements are stored:\t");
//    Serial.println(start / 8);
//  }
  // pinMode(LED, OUTPUT);
  // while(1)
  // {
  //   delay(1000);
  //   digitalWrite(LED,1);
  //   delay(1000);
  //   digitalWrite(LED,0);
  // }
}

void loop()
{
  
  if (millis() - t > TIME_INTERVAL)
  {
    t = millis();
    if (!digitalRead(SEND_DATA_UART_EN))
    {
      addr = 0x00;
      SPIFlash *flash;
      FlashBegin(flash,&flashOn);
      if(Serial.available())
        {
          String recivedData = Serial.readString();
        if(recivedData != "")
          {
            if(recivedData.substring(0,3) =="#P:")
            {
              //Change Preassure
              uint16_t preassure =  recivedData.substring(3,recivedData.indexOf("&")).toFloat();              
            }
            if(recivedData.substring(0,3) =="###")
            {
              //Clear Flash
              flash->eraseChip();
            }
          }
        }
        {
          uint32_t flashCapacity = flash->getCapacity();
          uint32_t flashTime,newFlashTime;
          while(addr<flashCapacity)
          {
            newFlashTime = flash->readLong(addr);
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
        }
        FlashEnd(flash,&flashOn);
        Serial.println("END");   
    }
    else 
    {
      float hight;
      {
        Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
        BMP280Init(&bmp);
        hight = bmp.readAltitude(1020);  
//        hight = bmp.readAltitude(ReadSavedPreassure()); // - initialHight;
      }
      SPIFlash *flash;
      FlashBegin(flash,&flashOn);
       flash->writeLong(addr, millis());
      addr += 4;
      flash->writeFloat(addr, hight);
      addr += 4;
      FlashEnd(flash,&flashOn);
    }
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

void BMP280Init(Adafruit_BMP280 * bmp)
{
  if (!bmp->begin())
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
  bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */
}

// uint8_t FindLastAdressEEM()
// {
//   for(int i = 0; i < EEPROM_BYTES / 2; i++)
//   {
//     if(eeprom_read_word(i) == 0xFFFF)
//     {
//       Serial.print("ADDR:\t");
//       Serial.print(i,HEX);
//       Serial.print("\tvalue\t");
//       Serial.println(eeprom_read_word(i - 1))
//       return (i - 1) * 2;
//     }
//   }
//   return (0xFF - 0x02);
// }

//uint16_t ReadSavedPreassure()
//{
//  return eeprom_read_word(seaLevelPreassureAdress);
//}
//
//void UpdatePreassure(uint16_t newPreassure)
//{
//  //seaLevelPreassureAdress++;
//  eeprom_update_word(seaLevelPreassureAdress, newPreassure);
//}


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
