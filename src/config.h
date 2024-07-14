#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
// --> D1, D2, D5, D6, D7 are safe. all others be careful with using

#define IS_REMOTE 1
#define IS_RECEIVER !IS_REMOTE

#define USE_ESPNOW 1
#define USE_WIFI 0 // to get mac address
#define USE_WII_NUNCHUCK 1

// motor drivers: 
#define USE_HOVER_SERIAL 0
#define USE_ESC 1

#define USE_SERIAL_MONITOR_CONTROL 1

#if IS_REMOTE


#define DEBUG_RX 0
#define DEBUG_WII_RAW 0
#define DEBUG_MAPPING 1
#define DEBUG_TX 1
#define DEBUG_FOR_PLOTTER 0

#else // IS_RECEIVER

#define DEBUG_RX 1
#define DEBUG_SERIAL_RECEIVE 1
#define DEBUG_TX 1

#endif

#define INVERT_CMD_LEFT_L 1 // = cmd_L for single board
#define INVERT_CMD_LEFT_R 0
#define INVERT_CMD_RIGHT_L 0 // = cmd_R for single board
#define INVERT_CMD_RIGHT_R 1
// #define INVERT_CMD_LEFT_L 0 // = cmd_L for single board
// #define INVERT_CMD_LEFT_R 1
// #define INVERT_CMD_RIGHT_L 1 // = cmd_R for single board
// #define INVERT_CMD_RIGHT_R 0

#define INVERT_SPEED_LEFT_L 1
#define INVERT_SPEED_LEFT_R 1
#define INVERT_SPEED_RIGHT_L 1
#define INVERT_SPEED_RIGHT_R 1

#define USE_CMD_L_R 1 // dont use steer/speed anymore - steer becomes L, speed becomes R
#define USE_DUAL_BOARDS 0

//uint8_t MAC_ADDRESS_REMOTE[6] = {0x08, 0x3A, 0x8D, 0xCC, 0xA7, 0x23}; // 08:3A:8D:CC:A7:23 
uint8_t MAC_ADDRESS_REMOTE[6] = {0x2C, 0xF4, 0x32, 0x12, 0xD7, 0x06}; // 2C:F4:32:12:D7:06 // with soldered battery wemos board

#if USE_HOVER_SERIAL
uint8_t MAC_ADDRESS_RECEIVER[6] = {0x08, 0x3A, 0x8D, 0xCC, 0x99, 0x46}; // 08:3A:8D:CC:99:46 in sofa now
//uint8_t MAC_ADDRESS_RECEIVER[6] = {0x08, 0x3A, 0x8D, 0xCC, 0xC5, 0xC3}; // 08:3A:8D:CC:C5:C3 // newer dual
#elif USE_ESC

// TEGEL 
// uint8_t MAC_ADDRESS_RECEIVER[6] = {0x08, 0x3A, 0x8D, 0xCC, 0xDC, 0x56};   // 08:3A:8D:CC:DC:56 test for 60V driver HoverPlatform

// uint8_t MAC_ADDRESS_RECEIVER[6] = {0x84, 0xF3, 0xEB, 0xBE, 0xE3, 0x34}; // left side = 84:F3:EB:BE:E3:34
// uint8_t MAC_ADDRESS_RECEIVER[6] = {0xC8, 0xC9, 0xA3, 0x2F, 0x9D, 0x4D}; // right side= C8:C9:A3:2F:9D:4D
uint8_t MAC_L[6] = {0x84, 0xF3, 0xEB, 0xBE, 0xE3, 0x34}; // left side = 84:F3:EB:BE:E3:34
uint8_t MAC_R[6] = {0xC8, 0xC9, 0xA3, 0x2F, 0x9D, 0x4D}; // right side= C8:C9:A3:2F:9D:4D
#endif



#endif // CONFIG_H


