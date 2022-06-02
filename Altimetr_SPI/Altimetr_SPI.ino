#include <avr/eeprom.h>
#include <Adafruit_BMP280.h>
#include <SPIMemory.h>
#include <SPI.h>
#include "lib/config.h"

/*
 * In this project I am using BMP280 and Flash
 * Both are using SPI to comminicate
 * The BMP280 library seems to be ok, but the 
 * SPIMemory causes some problems, when used with 
 * BMP280. To solve this problem when I finish 
 * using SPIFlash object I run the SPI.end() function
 */

//---------------UNO-------------------------------
#define CS_BMP 10
#define CS_FLASH 2
#define MOSI 11
#define MISO 12
#define SCK 13
#define SEND_DATA_UART_EN 7

#define BAUD_RATE 115200
//--------------GLOBAL VARIABLES--------------------
float initialHight;
uint32_t t = 0; // timer updated with millis()
bool flashOn = false;
uint32_t addr = START_ADRESS;
uint8_t seaLevelPreassureAdress = 0x00;
//--------------------------------------------------
//--------------------------------------------------

//--------------------------------------------------
//----------------FUNCTIONS-------------------------

//-------------------BMP----------------------------
void BMP280Init(Adafruit_BMP280 *bmp);

//-------------------EEPROM-------------------------
void UpdatePreassure(uint16_t newPreassure);
uint16_t ReadSavedPreassure();
// uint8_t FindLastAdressEEM();
//--------------------------------------------------

//--------------------FLASH-------------------------
bool CheckAdress(SPIFlash *flash, uint32_t addr);
void FindEmptyAdress(SPIFlash *flash, uint32_t *startingAdress);
//--------------------------------------------------
//--------------------------------------------------

void setup()
{
  //Set all pins as OUTPUT:
  for (int i = 0; i < 11; i++)
    pinMode(i, OUTPUT);
  //and one pin as an input to know, when PC is connected
  pinMode(SEND_DATA_UART_EN, INPUT_PULLUP);
  //--------------------------------------------------

  //UART INITIALISATION
  Serial.begin(BAUD_RATE);
  while (!Serial && !digitalRead(SEND_DATA_UART_EN));
  delay(100);
  if (Serial)
    Serial.println("Device connected");
  //--------------------------------------------------
  //EEPROM Test:
  {
    Serial.print("TST:Saved Preassure:\t");
    Serial.println(ReadSavedPreassure());
  }
  
  {
    SPIFlash flash(CS_FLASH);
    flash.begin();
    Serial.println("TST:FLASH1");
    delay(500);
    flash.writeLong(0x04,37);
    delay(500);
    Serial.print("TST:ADR 0x00:\t");
    delay(1000);
    long ad = flash.readLong(0x04);
    Serial.println(ad);
    SPI.end();
  }  
    
  //BMP280 TEST
  {
    Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
    BMP280Init(&bmp);
    initialHight = bmp.readAltitude(ReadSavedPreassure());
    if (Serial)
    {
      Serial.print("TST:INITIAL HIGHT:\t");
      Serial.println(initialHight);
      Serial.print("TST:TEMP:\t");
      Serial.println(bmp.readTemperature());
      Serial.print("TST:PRESSURE:\t");
      Serial.println(bmp.readPressure());
      
    }
  }
  //--------------------------------------------------

//  //FLASH TEST
  {
    Serial.println("TST:FLASH");
    delay(1000);
    SPIFlash flash(CS_FLASH);
    flash.begin();
  //  if(SEND_DATA_UART_EN)
  //  {
  //    flash.eraseChip();
  //    Serial.println("ChipErased");
  //  }
    delay(1000);
  //  if (Serial)
    {
      Serial.println("TST:FLASH1");
      delay(500);
  //    flash.writeLong(0x00,2137);
      delay(500);
      Serial.print("TST:ADR 0x00:\t");
      delay(1000);    
      long ad = flash.readLong(0x00);
      Serial.println(ad);
    }
  //  if (Serial)
    {
      Serial.print("TST:Read Flash:\t");
  //    Serial.println(flash.readFloat(0x00));
      unsigned long sizeF = flash.getCapacity();
      Serial.print("TST:Flash Memory has ");
      Serial.print(sizeF);
      Serial.println(" bytes.");
    }
  //  for (int i = 0; i < 2; i++)
  //  {
  //    Serial.print("TST:time:\t");
  //    Serial.print(flash.readLong(i));
  //    i++;
  //    Serial.print("\thight\t");
  //    Serial.println(flash.readLong(i));
  //    delay(500);
  //    
  //  }
  //
    delay(1000);
    uint32_t start = 0;
    FindEmptyAdress(&flash, &start);
  //  if (Serial)
    {
      Serial.print("TST:START ADDR:\t");
      Serial.println(start);
      Serial.print("TST:How many measurements are stored:\t");
      Serial.println(start / 8);
    }
    SPI.end();
  }
  Serial.println("SETUP ENDS");
}

void loop()
{
  if (!digitalRead(SEND_DATA_UART_EN))
  {
    if (Serial)
    {
      Serial.println("#CM");
      uint32_t t = millis();
      while (millis() - t < 2000)
        if (Serial.available() > 3)
        {
          String recivedData;
          while (Serial.available() > 3)
          {
            recivedData += Serial.readString();
            delay(50);
          }
          Serial.println(recivedData);
          Serial.println(" ");
          if (recivedData.length() > 3)
          {
            delay(100);
            if (recivedData.substring(0, 3) == "#P:")
            {
              //Change Preassure
              uint16_t preassure =  recivedData.substring(3, recivedData.indexOf("&")).toFloat();
              UpdatePreassure(preassure);
              Serial.print("TST:New Preassure");
              Serial.println(ReadSavedPreassure());
            }
            else if (recivedData.substring(0, 3) == "###")
            {
              SPIFlash flash(CS_FLASH);
              //Clear Flash
              flash.eraseChip();
              Serial.println("TST:CHip Erased");
              SPI.end();
            }
            else if (recivedData.substring(0, 3) == "#MD")
            {
              Serial.println("###MD!!!");
              SPIFlash flash(CS_FLASH);
              flash.begin();
              addr = 0x00;//to start with the first value in the flash
              {
                uint32_t flashCapacity;
                uint32_t flashTime, newFlashTime;

                do
                {
                  flashCapacity = flash.getCapacity();
                  Serial.println(flashCapacity);
                }while (flashCapacity < 1); // to avoid an error with reading the 0 capacity value from flash
                
                Serial.print("TST: Capaicty\t");
                Serial.println(flashCapacity);
                
                while (addr < 100)//flashCapacity
                {
                  //read all of the
                  newFlashTime = flash.readLong(addr);
                  if (flashTime > newFlashTime)
                    Serial.println("end");//the new series of measurements has started
                  flashTime = newFlashTime;
                  //Send the data
                  Serial.print("#t:");
                  Serial.print(newFlashTime);
                  addr += 4;
                  Serial.print("&h:");
                  Serial.print(flash.readFloat(addr));
                  Serial.println("$");
                  addr += 4;
                }
                SPI.end();
                Serial.println("END");
                delay(2000);
              }
            }
          }
        }
    }
  }

  else if (millis() - t > TIME_INTERVAL)
  {
    t = millis();
    //Just measure the hight and write it to flash
    Serial.println("MEASURE HIGHT");
    float hight;
    {
      delay(100);
      Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
      bmp.begin();
      Serial.println("MEASURE HIGHT1");
      delay(100);
      BMP280Init(&bmp);
      Serial.println("MEASURE HIGHT2");
      delay(100);
      hight = bmp.readAltitude(ReadSavedPreassure()); // - initialHight;
      Serial.println("MEASURE HIGHT3");
      delay(100);
    }
    Serial.println(hight);
    delay(100);
        
    delay(1000);
    if(Serial)
    {
      Serial.print("ADR:\t");
      Serial.print(addr);
      Serial.print("\ttime\t");
      Serial.print(millis());
      Serial.print("\tHight\t");
      Serial.println(hight);
    }
    SPIFlash flash(CS_FLASH);
    flash.begin();
    flash.writeLong(addr, millis());
    addr += 4;
    flash.writeFloat(addr, hight);
    addr += 4;
    SPI.end();
  }

  delay(1000);
}

//--------------------------------------------------
//--------------------BMP280------------------------
void BMP280Init(Adafruit_BMP280 * bmp)
{
  while (!bmp->begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
//    while (1)
      delay(1000);
  }
  /*
    Mode:           Normal . 0xF4 [1:0] . 11
    Oversampling:   Standard Resolution
    osrs_p:         x4 . 0xF4 [4:2] . 011
    osrs_t:         x1 . 0xF4 [7:5] . 001
    ODR[Hz]:        7.3
    RMS Noise[cm]:  6.4
    3 wire SPI:     false . 0xF5[0] . 0
    IIR filter:     4 . 0xF5[4:2] . 010?
    standby [ms]:   125 . 0xF5 [7:5] . 010
  */
  bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X4,     /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */
}

//--------------------------------------------------
//--------------------FLASH-------------------------

// checks if the adress is empty
bool CheckAdress(SPIFlash * flash, uint32_t addr)
{
  Serial.println("Check Adress IN");
  int32_t val = flash->readLong(addr);
  Serial.println(val);
   Serial.println("Check Adress OUT");
  return (val == 0xFFFFFFFF);
}

void FindEmptyAdress(SPIFlash * flash, uint32_t *startingAdress)
{
  Serial.println("Find Adress IN");

  uint32_t sizeF = flash->getCapacity();
  while (1)
  {
    if (*startingAdress > sizeF)
    {
      *startingAdress = 0;
      if (Serial)
      {
        Serial.print("FULL MEMORY, SIZE:\t");
        Serial.println(sizeF);
      }
      break;
    }
    if (CheckAdress(flash, *startingAdress))
      break;
    *startingAdress += 8;
  }

   Serial.println("Find Adress OUT");
}
//--------------------------------------------------

//--------------------------------------------------
//-------------------EEPROM-------------------------
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

uint16_t ReadSavedPreassure()
{
  return eeprom_read_word(seaLevelPreassureAdress);
}

void UpdatePreassure(uint16_t newPreassure)
{
  //seaLevelPreassureAdress++;
  eeprom_update_word(seaLevelPreassureAdress, newPreassure);
}


/*
  To do:

  .MIKRO
  Sprawdzenie całego programu

  .PC
  Komunikacja . suma kontrolna, arbitraż
  Może się da:
  Obliczanie ciśnienia na poziomie morza na podstawie aktualnej wysokości i wskazań z BMP280?
*/
