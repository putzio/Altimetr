#ifndef ALT_CONFIG_H
#define ALT_CONFIG_H
// // pinout
// // CS -> write LOW to choose the salve
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

#define EEPROM_BYTES 256

#define SEA_LEVEL_HPA 1020
#define TIME_INTERVAL 1000 // ms
#define START_ADRESS 0x00

#define F_SPI 1000000 // 1 MHz -> the same frequency is required, so we take the lower one from BMP and set it to FLASH
#endif