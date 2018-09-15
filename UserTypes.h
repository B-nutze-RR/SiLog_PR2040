#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"
#include "SdFat.h"
#include "FreeStack.h"
#include <RTClib.h>

#define console Serial
#define gps Serial1

#define HEARTBEAT 1000000
#define GPS_READ_INTERVAL 2000 // in microseconds
#define BAT_VOLT_RATE 1*1*1000*1000 // Mins*Secs*Millis*Micros

#define GPS_RTC_TIME_DIFF 1  // in seconds
#define GPS_RTC_TIME_ALIGN 500  // in microseconds
#define DS3231_TIME_ADDR  0x00

#define RTC_PPS_IRQ PA13  // EXTI13
#define GPS_PPS_IRQ PA8   // EXTI8
#define BAT_VOLT_PIN PB0  // Analog read pin

#define GPS_PPS_MASK 1
#define RTC_PPS_MASK 2
#define READ_GPS_MASK 4
#define BAT_VOLT_MASK 8
#define GPS_DATA_RECORD_MASK  0x0100
#define GPS_DATA_VALID_MASK   0x0200

#define LED_RED PB11
#define LED_GREEN PB10
#define LED_BLUE PB1
#define BOARD_BUTTON_PIN PB8

//#define FILE_BASE_NAME "mpuraw_UID_YYYY-MM-DD_HH-MM-SS"
#define FILE_BASE_NAME "mpu-001_"
#define LOG_TIME_PER_FILE 1800  // in seconds

struct data_t {
  uint32_t GPStime;
  uint32_t RTCtime; 
  uint16_t Status;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t temp;
  uint16_t batVolt;
};

bool acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
void FATdateTime(uint16_t* date, uint16_t* time);
void GetUnitID(char* unitID);
void GPS_Process();
void ledBlink(int led, int dly);
void GPS_SetMin();
void GPS_SetDebug();
void GPS_Menu();
void RTC_Menu();
void userSetup();
void SetLogEndTime();

#endif  // UserTypes_h
