#include <avr/eeprom.h>
#include <Adafruit_BMP280.h>
#include <SPIMemory.h>
#include <SPI.h>
#include "lib/config.h"

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
void FlashBegin(SPIFlash *flash, bool *flashState);
void FlashEnd(SPIFlash *flash, bool *flashState);
// checks if the adress is empty
bool CheckAdress(SPIFlash *flash, uint32_t addr, bool flashState);
void FindEmptyAdress(SPIFlash *flash, volatile uint32_t *startingAdress, bool flashState);
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
  Serial.begin(9600);
  while (!Serial && !digitalRead(SEND_DATA_UART_EN));
  if(Serial)
    Serial.println("Device connected");
  //--------------------------------------------------

  //BMP280 TEST
  {
    Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
    BMP280Init(&bmp);  
    initialHight = bmp.readAltitude(ReadSavedPreassure());
    if (Serial)
    {
      Serial.print("INITIAL HIGHT:\t");
      Serial.println(initialHight);
      Serial.print("TEMP:\t");
      Serial.println(bmp.readTemperature());
      Serial.print("PRESSURE:\t");
      Serial.println(bmp.readPressure());
    }
  }
  //--------------------------------------------------
  
  //FLASH TAST
  SPIFlash flash (CS_FLASH);
  flash.begin();
  if (Serial)
  {
    Serial.print("ADR 0x00:\t");
    Serial.println(flash.readLong(0x00));
  }
   if (Serial)
   {
     Serial.print("Read Flash:\t");
     Serial.println(flash.readFloat(0x00));
     unsigned long sizeF = flash.getCapacity();
     Serial.print("Flash Memory has ");
     Serial.print(sizeF);
     Serial.println(" bytes.");
   }
   for(int i = 0; i < 64; i++)
   {
    Serial.print("time:\t");
    Serial.print(flash.readLong(i));
    i++;
    Serial.print("\thight\t");
    Serial.println(flash.readLong(i));
    delay(500);
   }

  volatile uint32_t start;
  FindEmptyAdress(flash, &start,flashOn);
  FlashEnd(flash, &flashOn);
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
    {//if the device is connected - read data from flash, write new preassure or erease flash        
      SPIFlash *flash;
      FlashBegin(flash,&flashOn);
      if(Serial.available())
        {
          String recivedData = Serial.readString();
          if(recivedData.length()>3)
          {
            if(recivedData.substring(0,3) =="#P:")
            {
              //Change Preassure
              uint16_t preassure =  recivedData.substring(3,recivedData.indexOf("&")).toFloat();        
              UpdatePreassure(preassure);
            }
            if(recivedData.substring(0,3) =="###")
            {
              //Clear Flash
              flash->eraseChip();
            }
          }
        }
        addr = 0x00;//to start with the first value in the flash
        {
          uint32_t flashCapacity;
          uint32_t flashTime,newFlashTime;

          while(flashCapacity<1)// to avoid an error with reading the 0 capacity value from flash
          {
             flashCapacity = flash->getCapacity();
          }
          while(addr<flashCapacity)
          {
            //read all of the 
            newFlashTime = flash->readLong(addr);
            if(flashTime>newFlashTime)
              Serial.println("end");//the new series of measurements has started
//            if(flashTime == newFlashTime) //if there was only one measurement during the flight??? 
//              {
//                //no new values
//                Serial.println("END");
//                delay(10000);
//                break;
//              }
            flashTime = newFlashTime;
            //Send the data
            Serial.print("\t#t:\t");
            Serial.print(newFlashTime);
            addr += 4;
            Serial.print("\t&h:\t");
            Serial.print(flash->readFloat(addr));
            Serial.println("$");
            addr += 4;        
          } 
          Serial.println("END");
                delay(10000);     
        }
        FlashEnd(flash,&flashOn);           
    }
    else 
    {//Just measure the hight and write it to flash
      float hight;
      {
        Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
        BMP280Init(&bmp);
        hight = bmp.readAltitude(ReadSavedPreassure()); // - initialHight;
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

//--------------------------------------------------
//--------------------BMP280------------------------
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

//--------------------------------------------------
//--------------------FLASH-------------------------
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
bool CheckAdress(SPIFlash *flash, uint32_t addr, bool flashState)
{
 if (!flashState)
 {
   FlashBegin(flash, &flashOn);
   if (Serial.available())
   {
     Serial.println("Initialised Flash in CheckAdress() function");
   }
 }
  int32_t val = flash->readLong(addr);
 if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
 {
   FlashEnd(flash, &flashOn);
 }
  return (val == -1);
}

void FindEmptyAdress(SPIFlash *flash, volatile uint32_t *startingAdress, bool flashState)
{
 if (!flashState)
 {
   FlashBegin(flash, &flashOn);
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
    if (CheckAdress(flash, *startingAdress, flashOn))
      break;
    *startingAdress += 8;
  }
 if (!flashState) // this var has not changedif FlashBegin(), because it is not a *ptr
 {
   FlashEnd(flash, &flashOn);
 }
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

->MIKRO
Sprawdzenie całego programu

->PC
Komunikacja -> suma kontrolna, arbitraż
Może się da:
Obliczanie ciśnienia na poziomie morza na podstawie aktualnej wysokości i wskazań z BMP280?
*/
