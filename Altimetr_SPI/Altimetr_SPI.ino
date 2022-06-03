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
//--------------------------------------------------
#define EMPTY_LONG_FLASH 0xFFFFFFFF
#define DEFAULT_PREASSURE 1013
#define BAUD_RATE 115200
//--------------GLOBAL VARIABLES--------------------
uint32_t t = 0; // timer updated with millis()
bool flashOn = false;
uint32_t addr;
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
void WaitForUartOK()
{
  while(Serial.read() != (int)'K');
}



void setup()
{
/*
 * 1. Inicjalizacja peryferiów                ok
 * 2. EEPROM                                  test
 * 3. Test BMP280                             test
 * 4. Test Flasha (połączenie - JEDEC???)
 * 5. Znalezienie pierwszego wolnego adresu->addr   test
 * 6. Pamięć w połowie pełna - dioda się świeci
 */  
  //Set all pins as OUTPUT:
  for (int i = 0; i < 11; i++)
    pinMode(i, OUTPUT);
  //and one pin as an input to know, when PC is connected
  pinMode(SEND_DATA_UART_EN, INPUT_PULLUP);
  //--------------------------------------------------

  //UART INITIALISATION
  Serial.begin(BAUD_RATE);
  while (!Serial && !digitalRead(SEND_DATA_UART_EN));
//  delay(100);
  if (Serial)
    Serial.println("Device connected");
  //--------------------------------------------------
  //EEPROM Test:
  if(Serial)
  {
    uint16_t preassure = ReadSavedPreassure();
    if(preassure<800||preassure>2000)
    {
      Serial.print("TST:Saved sea level preassure: ");
      Serial.print(preassure);
      Serial.print(" is out of range <800,2000> and will be changed to the default: ");
      Serial.println(DEFAULT_PREASSURE); 
      UpdatePreassure(DEFAULT_PREASSURE);
    }
    Serial.print("TST:Saved sea level preassure:\t");
    Serial.println(ReadSavedPreassure());
  }
  else if (ReadSavedPreassure()<900)
    UpdatePreassure(DEFAULT_PREASSURE);
  //BMP280 TEST
  {
    Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
    BMP280Init(&bmp);
    if (Serial)
    {
      Serial.print("TST:HIGHT:\t");
      Serial.println(bmp.readAltitude(ReadSavedPreassure()));
      Serial.print("TST:TEMP:\t");
      Serial.println(bmp.readTemperature());
      Serial.print("TST:PRESSURE:\t");
      Serial.println(bmp.readPressure());      
    }
  }    

//  //FLASH TEST
  {
    Serial.println("TST:FLASH");
    delay(1000);
    SPIFlash flash(CS_FLASH);
    flash.begin();
    delay(1000);
    if (Serial)
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
    
    FindEmptyAdress(&flash, &addr);
    unsigned long sizeF = flash.getCapacity();

    if(sizeF/addr < 2)
      digitalWrite(LED,1);
    
    if (Serial)
    {      
      Serial.print("TST:Flash Memory has ");
      Serial.print(sizeF);
      Serial.println(" bytes.");
      delay(1000);
      
      Serial.print("TST:START ADDR:\t");
      Serial.println(addr);
      Serial.print("TST:How many measurements are stored:\t");
      Serial.println(addr / 8);
    }
    SPI.end();
  }
  Serial.println("SETUP ENDS");
}

void loop()
{
  /*
   * 1. Sprawdzenie zworki - UART
   * 2. Jeżeli jest:
   * Obsłużenie akcji dla użytkownika:
   * Zmaina ciśnienia w EEPROM
   * Czyszczenie flasha
   * Wysyłanie danych po uarcie
   * 
   * 3. Jeżeli nie jest
   * Odczyty z czujnika i zapis do flasha
   */
  if (!digitalRead(SEND_DATA_UART_EN))
  {
    if (Serial)
    {
      Serial.println("#CM");
      uint32_t t = millis();
      while (millis() - t < 2000)
        if (Serial.available() > 1)//At least 2 characters
        {
          String recivedData;
          recivedData = Serial.readString();
          Serial.println(recivedData);
          Serial.println(" ");
//          for(int i = 0; i < sizeof(recivedData)/sizeof(recivedData[0]); i++)
//          {
//            Serial.println(recivedData[i],HEX);
//          }
          if (recivedData[0] = '#')
          {
            delay(100);
            if (recivedData[1] == 'P')
            {
              //Change Preassure
              uint16_t preassure =  recivedData.substring(3, recivedData.indexOf("&")).toFloat();
              UpdatePreassure(preassure);
              Serial.print("TST:New Preassure");
              Serial.println(ReadSavedPreassure());
            }
            else if (recivedData[1] == "#")
            {
              SPIFlash flash(CS_FLASH);
              //Clear Flash
              flash.eraseChip();
              Serial.println("TST:CHip Erased");
              SPI.end();
            }
            else if (recivedData[1] == 'M')
            {
              SPIFlash flash(CS_FLASH);
              flash.begin();
              addr = 0x00;//to start with the first value in the flash
              {
                uint32_t flashCapacity, flashTime = 0, newFlashTime;
                flashCapacity = flash.getCapacity();
                
                while (addr < flashCapacity)//flashCapacity
                {
                  //read all of the
                  uint32_t ti [6];
                  newFlashTime = flash.readLong(addr);
                  if(newFlashTime == EMPTY_LONG_FLASH)
                  {
                    Serial.println("END&");
                    WaitForUartOK();
                    break;
                  }
                  if (flashTime > newFlashTime)
                    {
                      Serial.println("end&");//the new series of measurements has started
                      WaitForUartOK();
                    }
                  flashTime = newFlashTime;
                  //Send the data
                  String message = "#t:";
                  message += newFlashTime;
                  addr += 4;
                  message += "&h:";
                  message += flash.readLong(addr);
                  message += "$";
                  Serial.print(message);
                  addr += 4;
                  WaitForUartOK();
                }
                SPI.end();
                
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
    uint32_t hight;
    {
      delay(100);
      Adafruit_BMP280 bmp(CS_BMP, MOSI, MISO, SCK);
      bmp.begin();
      delay(100);
      BMP280Init(&bmp);
      delay(100);
      hight = (uint32_t)bmp.readAltitude(ReadSavedPreassure()); // - initialHight;
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
    flash.writeLong(addr, hight);
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
//  Serial.println("Check Adress IN");
  int32_t val = flash->readLong(addr);
//  Serial.println(val);
//   Serial.println("Check Adress OUT");
  return (val == 0xFFFFFFFF);
}

void FindEmptyAdress(SPIFlash * flash, uint32_t *startingAdress)
{
//  Serial.println("Find Adress IN");

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

//   Serial.println("Find Adress OUT");
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
