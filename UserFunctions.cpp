// User data functions.  Modify these functions for your data items.
#include "UserTypes.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MicroNMEA.h>

//------------------------------------------------------------------------------

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
DS3231 rtc;
MPU6050 mpu(0x69);
HardwareTimer timer_ReadGPS(1);
HardwareTimer timer_BatVolt(2);
DateTime RTC_now;
DateTime GPS_now, StartRTCLogTime, tempDT;
//DateTime LogStart_time;
DateTime Last_RTC;
uint32_t LogEnd_time;
uint32_t CurrentDataReads = 0;

static uint32_t startMicros;
uint32_t tms = 0;

volatile uint16_t lastBatVolt = 0;
volatile uint32_t counter_GPSTrig = 0;
volatile uint32_t counter_BatVolt = 0;
volatile uint32_t counter_ReadGPS = 0;
volatile uint32_t timepulse_GPS = 0;
volatile uint32_t timepulse_RTC = 0;
volatile int32_t time_tmp = 0;
char buf_RTC[20];

//==========================================================
//  PROGRAM FLAGS
//==========================================================
volatile uint16_t GPSppsTriggered = 0;
volatile uint16_t RTCppsTriggered = 0;
volatile uint16_t ReadGPSTriggered = 0;
volatile uint16_t BatVoltTriggered = 0;
volatile uint16_t StartNewFileTriggered = 0;
volatile uint8_t  RTCBusy = 0;


void GPS_Process();

//==========================================================
//  INTERRUPT HANDLERS
//==========================================================
void GPSppsHandler(void)
{
  uint8_t irqState = digitalRead(GPS_PPS_IRQ);
  GPSppsTriggered = GPS_PPS_MASK & irqState;

  if (irqState) {
    counter_GPSTrig++;
  }
  digitalWrite(LED_BLUE, GPSppsTriggered);
}

void RTCppsHandler(void)
{
  RTCppsTriggered = RTC_PPS_MASK;
#ifndef HEARTBEAT
  digitalWrite(LED_RED, counter_BatVolt & 1);
#endif
  if (counter_BatVolt == 0)
    timer_BatVolt.refresh();
  counter_BatVolt++;
}

void BatVoltHandler(void)
{
  BatVoltTriggered = BAT_VOLT_MASK;
}

void ReadGPSHandler(void)
{
  ReadGPSTriggered = READ_GPS_MASK;
}

//==========================================================
//  ACQUIRE DATA RECORD
//==========================================================
bool acquireData(data_t* data) {
  //  data->GPStime = GPS_now.unixtime();
  data->GPStime = micros();
  data->RTCtime = rtc.now().unixtime();
  //  data->RTCtime = RTC_now.unixtime();

  data->Status = GPSppsTriggered | RTCppsTriggered | BatVoltTriggered;


  mpu.getMotion6(&data->ax, &data->ay, &data->az,
                 &data->gx, &data->gy, &data->gz);

  //  data->temp = mpu.getTemperature() / 340 + 36.53; //

  data->temp = mpu.getTemperature();

  if (BatVoltTriggered != 0) {
    lastBatVolt = analogRead(BAT_VOLT_PIN);
    counter_BatVolt = 0;
  }
  data->batVolt = lastBatVolt;
  //  console.println(counter_BatVolt);
  //  GPSppsTriggered = 0;

  RTCppsTriggered = BatVoltTriggered = 0;

#ifdef HEARTBEAT
  if (micros() >= tms) {
    tms = micros() + HEARTBEAT;
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
#endif

  CurrentDataReads++;

  if (data->RTCtime < LogEnd_time) {
    return true;
  } else {
    return false;
  }
}

void SetLogEndTime() {
  StartRTCLogTime = rtc.now();
  LogEnd_time = StartRTCLogTime.unixtime() + LOG_TIME_PER_FILE;
  CurrentDataReads = 0;
}

void GPS_Process() {
  bool validmsg = false;
  uint32_t tout;

  nmea.clear();
  tout = millis() + 5000;

  do {
    while (gps.available() && !validmsg) {
      char c = gps.read();
      validmsg = nmea.process(c);
    }
    console.println("=======");
    console.println(nmea.getSentence());
    console.println(nmea.getMessageID());
  } while  ((nmea.getMessageID()[0] != 'R') && (millis() < tout));

  GPS_now.setyear(nmea.getYear()); GPS_now.setmonth(nmea.getMonth()); GPS_now.setday(nmea.getDay());
  GPS_now.sethour(nmea.getHour()); GPS_now.setminute(nmea.getMinute()); GPS_now.setsecond(nmea.getSecond());
  //  console.println(nmea.getYear());
  //  console.println(GPS_now.year());
}

void RTC_AdjustToGPS() {
  GPS_Process();
  digitalWrite(LED_GREEN, HIGH);

  if ((abs(GPS_now.unixtime() - rtc.now().unixtime()) >= GPS_RTC_TIME_DIFF) && (GPS_now.year() != 2048) && nmea.isValid()) {
    rtc.adjust(GPS_now);
    console.println(F("RTC is adjustet to GPS."));
  } else {
    console.println(F("RTC is not adjustet to GPS (no fix).\nYou may leave it as is or set it manually from the menu."));
  }

  strncpy(buf_RTC, "YYYY.MM.DD hh:mm:ss\0", 20);
  console.print(F("RTC: ")); //RTC
  RTC_now = rtc.now();
  console.println(RTC_now.format(buf_RTC));

  digitalWrite(LED_GREEN, LOW);
}
//==========================================================
//  HARDWARE HUMAN INTERFACE
//==========================================================
void ledBlink(int led, int dly) {

  digitalWrite(led, HIGH);
  delay(dly);
  digitalWrite(led, LOW);
  //      delay(dly);
}


//void ledTest() {
//  ledBlink (LED_GREEN, 200);
//  ledBlink (LED_RED, 200);
//}

//==========================================================
//  GPS CONFIG
//==========================================================
void GPS_SetMin() {
  MicroNMEA::sendSentence(gps, "$PUBX,40,GLL,0,0,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,VTG,0,0,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GSA,0,0,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GSV,0,0,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GGA,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,RMC,0,1,0,0,0,0");
}

void GPS_SetDebug() {
  MicroNMEA::sendSentence(gps, "$PUBX,40,GLL,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,VTG,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GSA,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GSV,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,GGA,0,1,0,0,0,0");
  MicroNMEA::sendSentence(gps, "$PUBX,40,RMC,0,1,0,0,0,0");
}

//==========================================================
//  SETUP
//==========================================================
void userSetup() {
  //==========================================================
  //  INDICATION INITIALIZATION
  //==========================================================
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

  gps.begin(9600);
  // ------   Disable or enable certain messages (Disabling is useful to conserve time for processing)
  GPS_SetMin();

  uint32_t tout = millis();
  while (!gps.available() && ((millis() - tout) < 1000) ) { }
  if ((millis() - tout) < 1000) {
    delay(500);
    ledBlink(LED_BLUE, 1000);
    delay(500);
    ledBlink(LED_BLUE, 1000);
  }

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  rtc.begin(); // RTC initialization

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }

  rtc.write(0x0E, 0); // RTC 1 Hz pulse

  mpu.initialize();
  if (mpu.getDeviceID() > 0) {
    //    console.print(F("MPU ID: ")); console.println(mpu.getDeviceID());
    delay(500);
    ledBlink(LED_RED, 1000);
    delay(500);
    ledBlink(LED_RED, 1000);
  }

  pinMode(GPS_PPS_IRQ, INPUT);
  attachInterrupt(GPS_PPS_IRQ, GPSppsHandler, CHANGE);

  pinMode(RTC_PPS_IRQ, INPUT_PULLUP); // RTC pulse is LOW active
  attachInterrupt(RTC_PPS_IRQ, RTCppsHandler, FALLING);


  pinMode(BAT_VOLT_PIN, INPUT_ANALOG); // ADC pin for battery monitoring



  //==========================================================
  //  BATTERY VOLTAGE TIMER INITIALIZATION
  //==========================================================
  timer_BatVolt.pause();
  // Set up period
  timer_BatVolt.setPeriod(BAT_VOLT_RATE); // in microseconds
  // Set up an interrupt on channel 1
  timer_BatVolt.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer_BatVolt.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer_BatVolt.attachCompare1Interrupt(BatVoltHandler);
  // Refresh the timer's count, prescale, and overflow
  timer_BatVolt.refresh();


  //==========================================================
  //  GPS READ DATA TIMER INITIALIZATION
  //==========================================================
  /*
    timer_ReadGPS.pause();
    // Set up period
    timer_ReadGPS.setPeriod(GPS_READ_INTERVAL); // in microseconds
    // Set up an interrupt on channel 1
    timer_ReadGPS.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer_ReadGPS.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    timer_ReadGPS.attachCompare1Interrupt(ReadGPSHandler);
    // Refresh the timer's count, prescale, and overflow
    timer_ReadGPS.refresh();
    // Start the timer counting
    //  timer_ReadGPS.resume();
  */
  RTC_AdjustToGPS();

  // Start the BatVolt timer counting
  timer_BatVolt.resume();
}

void GPS_Debug(uint8_t isDebug) {
  GPS_SetDebug();
  while (!isDebug || !console.available()) {
    if (console.available()) {      // If anything comes in Serial (USB),
      gps.write(console.read());   // read it and send it out Serial1 (pins 0 & 1)
    }

    if (gps.available()) {     // If anything comes in Serial1 (pins 0 & 1)
      console.write(gps.read());   // read it and send it out Serial (USB)
    }
  }
  GPS_SetMin();
}

void GPS_Menu() {
  uint8_t ext = 0;

  while (!ext) {
    do {
      delay(10);
    } while (console.available() && console.read() >= 0);
    console.println();
    console.println(F("GPS menu:"));
    console.println(F("d - Debug mode"));
    console.println(F("c - Config mode"));
    console.println(F("x - exit"));
    while (!console.available()) {
      yield();
    }
    char c = tolower(console.read());

    // Discard extra Serial data.
    do {
      delay(10);
    } while (console.available() && console.read() >= 0);

    if (c == 'd') {
      console.println(F("\nGPS Debug - type any character to stop\n"));
      GPS_Debug(true);
    } else if (c == 'c') {
      console.println(F("\nGPS Config - RESET to stop\n"));
      GPS_Debug(false);;
    } else if (c == 'x') {
      ext = 1;
    } else {
      console.println(F("Invalid entry"));
    }
  }
}

void RTC_Show() {
  console.println(F("\nRTC Ddebug - type any character to stop\n"));

  while (!console.available()) {
    //        GPS_Process();
    if (RTCppsTriggered) {
      //      char buf_RTC[100];
      strncpy(buf_RTC, "YYYY.MM.DD hh:mm:ss\0", 20);
      console.print(F("\nRTC: ")); //RTC
      RTC_now = rtc.now();
      console.println(RTC_now.format(buf_RTC));
      RTCppsTriggered = 0;
    }
    //        char buf_GPS[100];
    //        strncpy(buf_GPS, "DD.MM.YYYY  hh:mm:ss\0", 100);
    //        console.print(F("GPS: ")); //GPS
    //        console.println(GPS_now.format(buf_GPS));
    //    delay(1000);
  }
}

void RTC_Console_Set() {
  byte Year, Month, Day, Hour, Minute, Second;
  boolean GotString = false;
  char InChar;
  byte Temp1, Temp2;
  char InString[19];
  char Date[10];
  char Time[10];
  byte j = 0;

  console.println(F("\nEnter UTC Date and Time in this format: 27-11-2017 12:34:56\n"));

  while (!GotString) {
    if (console.available()) {
      InChar = console.read();
      if (InChar == 0x0D)  {        // Carriage return +
        if (console.read() == 0x0A) // +Line feed (CRLF)
          GotString = true;
      }
      else {
        InString[j] = InChar;
        j += 1;
      }
    }
  }
  InString[j] = '\0';
  console.println(InString);

  if (j != 19) {
    console.println(F("\nWrong input!\n"));
    return;
  }

  strncpy(Date, InString, 10); Date[10] = '\0';
  strncpy(Time, &InString[11], 8); Time[8] = '\0';

  //  console.println(Date);
  //  console.println(Time);

  rtc.adjust(DateTime(Date, Time));
}

void RTC_Menu() {
  uint8_t ext = 0;

  while (!ext) {
    do {
      delay(10);
    } while (console.available() && console.read() >= 0);
    console.println();
    console.println(F("RTC menu:"));
    console.println(F("g - show UTC time"));
    //    console.println(F("p - sync RTC to GPS time"));
    console.println(F("s - set manually UTC time"));
    console.println(F("x - exit"));
    while (!console.available()) {
      yield();
    }
    char c = tolower(console.read());

    // Discard extra Serial data.
    do {
      delay(10);
    } while (console.available() && console.read() >= 0);

    if (c == 'g') {
      RTC_Show();
    } else if (c == 's') {
      RTC_Console_Set();
    } else if (c == 'p') {
      RTC_AdjustToGPS();
    } else if (c == 'x') {
      ext = 1;
    } else {
      console.println(F("Invalid entry"));
    }
  }
}

void GetUnitID(char* unitID) {
  //  *unitID = "001";
}

void FATdateTime(uint16_t* date, uint16_t* time) {
  // User gets date and time from GPS or real-time
  // clock in real callback function
  //  while (RTCBusy) {
  //    yield();
  //  }
  //  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(rtc.now().year(), rtc.now().month(), rtc.now().day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(rtc.now().hour(), rtc.now().minute(), rtc.now().second());
}

// Print a data record.
void printData(Print * pr, data_t* data) {
  /*if (startMicros == 0) {
    startMicros = data->RTCtime;
    }*/
  pr->print(data->GPStime);
  pr->write(',');
  //  pr->print(data->RTCtime);
  tempDT = data->RTCtime;
  strncpy(buf_RTC, "YYYY-MM-DD hh:mm:ss\0", 20);
  pr->print(tempDT.format(buf_RTC));
  pr->write(',');
  pr->print(data->Status);
  pr->write(',');
  pr->print(data->ax);
  pr->write(',');
  pr->print(data->ay);
  pr->write(',');
  pr->print(data->az);
  pr->write(',');
  pr->print(data->gx);
  pr->write(',');
  pr->print(data->gy);
  pr->write(',');
  pr->print(data->gz);
  pr->write(',');
  pr->print(data->temp / 340 + 37);  // last number must be 36.53 but we presume to use 37 for rounding
  pr->write(',');
  pr->println(data->batVolt);
}

// Print data header.
void printHeader(Print * pr) {
  //  pr->println(BAT_VOLT_RATE);
  pr->println(F("Station_code\tMPU-001"));
  pr->println(F("Sampling_rate\t100.0000"));
  pr->print(F("Start_date\t")); strncpy(buf_RTC, "DD.MM.YYYY\0", 20); pr->println(StartRTCLogTime.format(buf_RTC));
  pr->print(F("Start_time\t")); strncpy(buf_RTC, "hh:mm:ss\0", 20); pr->println(StartRTCLogTime.format(buf_RTC));
  pr->println(F("GPSTime,RTCTime,Status,ax,ay,az,gx,gy,gz,temp,batVolt"));
}
