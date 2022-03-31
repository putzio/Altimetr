# 1 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
# 2 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 2
# 3 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 2
# 4 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 2
# 5 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 2
# 6 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 2

// pinout
// CS -> write LOW to choose the salve
# 24 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
SerialFlashFile flash;
Adafruit_BMP280 bmp(7 /* PB0*/, 8 /* PA1*/, 9 /* PA2*/, 10 /* PA3*/);

float maxHightEEM 
# 27 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 3
                 __attribute__((section(".eeprom")))
# 27 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
                      ;
float maxHightRAM;
float initialHight;
uint64_t t = 0; // timer updated with millis()
void setup()
{
  // put your setup code here, to run once:
  for (int i = 0; i < 11; i++) // For all pins
    pinMode(i, 1 /* used for pinMode() */);
  pinMode(3 /* INT - PA7*/, 2 /* used for pinMode() */);
  Serial.begin(115200);
  Serial.println("START");

  if (!bmp.begin())
  {
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 42 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 42 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
                  "Could not find a valid BMP280 sensor, check wiring or " "try a different address!"
# 42 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino" 3
                  ); &__c[0];}))
# 42 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
                  ))
                                                );
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
# 58 "g:\\Studia\\PUT Rocket LAB\\altimetr\\Altimetr_SPI\\Altimetr_SPI.ino"
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1, /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4, /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4, /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

  initialHight = bmp.readAltitude(1020.0);
  if (Serial.available())
  {
    Serial.print("INITIAL HIGHT:\t");
    Serial.println(initialHight);
  }

  // Flash
  if (!SerialFlash.begin(1 /* PA5*/))
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
    if (millis() - t > 1000 /* ms*/)
    {
      t = millis();
      float hight = bmp.readAltitude(1020.0) - initialHight;
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

      if (Serial.available() && !digitalRead(3 /* INT - PA7*/))
      {
        float readHight = eeprom_read_float(&maxHightEEM);
        Serial.print("Max Hight:\t");
        Serial.println(readHight);
      }
    }
  }
