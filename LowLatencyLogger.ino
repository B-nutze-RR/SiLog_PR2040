/**
   This program logs data to a binary file.  Functions are included
   to convert the binary file to a csv text file.

   Samples are logged at regular intervals.  The maximum logging rate
   depends on the quality of your SD card and the time required to
   read sensor data.  This example has been tested at 500 Hz with
   good SD card on an Uno.  4000 HZ is possible on a Due.

   If your SD card has a long write latency, it may be necessary to use
   slower sample rates.  Using a Mega Arduino helps overcome latency
   problems since 12 512 byte buffers will be used.

   Data is written to the file using a SD multiple block write command.
*/
#include <SPI.h>

#include "UserTypes.h"

#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define console MinSerial
#endif  // __AVR_ATmega328P__
//==============================================================================
// Start of configuration constants.
//==============================================================================
// Abort run on an overrun.  Data before the overrun will be saved.
#define ABORT_ON_OVERRUN 1
//------------------------------------------------------------------------------
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC = 10000;
//------------------------------------------------------------------------------
// Set USE_SHARED_SPI non-zero for use of an SPI sensor.
// May not work for some cards.
#ifndef USE_SHARED_SPI
#define USE_SHARED_SPI 0
#endif  // USE_SHARED_SPI
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = PA4;
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for
// overrun errors and logging continues unless ABORT_ON_OVERRUN
// is non-zero.
#ifdef ERROR_LED_PIN
#undef ERROR_LED_PIN
#endif  // ERROR_LED_PIN
const int8_t ERROR_LED_PIN = PB1;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 256000;
//
// log file base name if not defined in UserTypes.h
#ifndef FILE_BASE_NAME
#define FILE_BASE_NAME "data"
#endif  // FILE_BASE_NAME
//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT-1 additional
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of ten 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 10;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 2;
//
#elif RAMEND < 0X20FF
// Use total of four 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 12 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME FILE_BASE_NAME "##.bin"

// Size of file base name.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
char binName[FILE_NAME_DIM] = FILE_BASE_NAME "00.bin";

SdFat sd(1);

SdBaseFile binFile;

bool StopLogging = false;

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4) / sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM * sizeof(data_t);

struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};
//==============================================================================
// Error messages stored in flash.
#define error(msg) {sd.errorPrint(&console, F(msg));fatalBlink();}
//------------------------------------------------------------------------------
void fatalBlink() {
  while (true) {
    SysCall::yield();
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}
//------------------------------------------------------------------------------
// read data file and check for overruns
void checkOverrun() {
  bool headerPrinted = false;
  block_t block;
  uint32_t bn = 0;

  if (!binFile.isOpen()) {
    console.println();
    console.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  console.println();
  console.print(F("FreeStack: "));
  console.println(FreeStack());
  console.println(F("Checking overrun errors - type any character to stop"));
  while (binFile.read(&block, 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      if (!headerPrinted) {
        console.println();
        console.println(F("Overruns:"));
        console.println(F("fileBlockNumber,sdBlockNumber,overrunCount"));
        headerPrinted = true;
      }
      console.print(bn);
      console.print(',');
      console.print(binFile.firstBlock() + bn);
      console.print(',');
      console.println(block.overrun);
    }
    bn++;
  }
  if (!headerPrinted) {
    console.println(F("No errors found"));
  } else {
    console.println(F("Done"));
  }
}
//-----------------------------------------------------------------------------
// Convert binary file to csv file.
void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t block;
  uint32_t t0 = millis();
  uint32_t syncCluster = 0;
  SdFile csvFile;
  char csvName[FILE_NAME_DIM];

  if (!binFile.isOpen()) {
    console.println();
    console.println(F("No current binary file"));
    return;
  }
  console.println();
  console.print(F("FreeStack: "));
  console.println(FreeStack());

  // Create a new csvFile.
  strcpy(csvName, binName);
  strcpy(&csvName[BASE_NAME_SIZE + 3], "csv");

  if (!csvFile.open(csvName, O_WRITE | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  binFile.rewind();
  console.print(F("Writing: "));
  console.print(csvName);
  console.println(F(" - type any character to stop"));
  printHeader(&csvFile);
  uint32_t tPct = millis();
  while (!console.available() && binFile.read(&block, 512) == 512) {
    uint16_t i;
    if (block.count == 0 || block.count > DATA_DIM) {
      break;
    }
    if (block.overrun) {
      csvFile.print(F("OVERRUN,"));
      csvFile.println(block.overrun);
    }
    for (i = 0; i < block.count; i++) {
      printData(&csvFile, &block.data[i]);
    }
    if (csvFile.curCluster() != syncCluster) {
      csvFile.sync();
      syncCluster = csvFile.curCluster();
    }
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        console.print(pct, DEC);
        console.println('%');
      }
    }
    if (console.available()) {
      break;
    }
  }
  csvFile.close();
  console.print(F("Done: "));
  console.print(0.001 * (millis() - t0));
  console.println(F(" Seconds"));
}
//-----------------------------------------------------------------------------
void createBinFile() {
  // max number of blocks to erase per erase call
  const uint32_t ERASE_SIZE = 262144L;
  uint32_t bgnBlock, endBlock;

  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) {
    console.println(F("Deleting tmp file " TMP_FILE_NAME));
    if (!sd.remove(TMP_FILE_NAME)) {
      error("Can't remove tmp file");
    }
  }
  // Create new file.
  console.println(F("\nCreating new file"));
  binFile.close();
  if (!binFile.createContiguous(TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  // Flash erase all data in the file.
  console.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) {
      endErase = endBlock;
    }
    if (!sd.card()->erase(bgnErase, endErase)) {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
}
//------------------------------------------------------------------------------
// dump data file to Serial
void dumpData() {
  block_t block;
  if (!binFile.isOpen()) {
    console.println();
    console.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  console.println();
  console.println(F("Type any character to stop"));
  delay(1000);
  printHeader(&console);
  while (!console.available() && binFile.read(&block , 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      console.print(F("OVERRUN,"));
      console.println(block.overrun);
    }
    for (uint16_t i = 0; i < block.count; i++) {
      printData(&console, &block.data[i]);
    }
  }
  console.println(F("Done"));
}
//------------------------------------------------------------------------------
// log data
void logData() {
  digitalWrite(LED_RED,LOW);
  StopLogging = false;
  do {
    createBinFile();
    recordBinFile();
    renameBinFile();
  } while (!StopLogging);
  digitalWrite(LED_RED,HIGH);
}
//------------------------------------------------------------------------------
void openBinFile() {
  char name[FILE_NAME_DIM];
  strcpy(name, binName);
  console.println(F("\nEnter two digit version"));
  //  console.write(name, BASE_NAME_SIZE);
  for (int i = 0; i < 2; i++) {
    while (!console.available()) {
      SysCall::yield();
    }
    char c = console.read();
    console.write(c);
    if (c < '0' || c > '9') {
      console.println(F("\nInvalid digit"));
      return;
    }
    name[BASE_NAME_SIZE + i] = c;
  }
  console.println(&name[BASE_NAME_SIZE + 2]);
  if (!sd.exists(name)) {
    console.println(F("File does not exist"));
    return;
  }
  binFile.close();
  strcpy(binName, name);
  if (!binFile.open(binName, O_READ)) {
    console.println(F("open failed"));
    return;
  }
  console.println(F("File opened"));
}
//------------------------------------------------------------------------------
void recordBinFile() {
  const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;
  // Index of last queue location.
  const uint8_t QUEUE_LAST = QUEUE_DIM - 1;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT - 1];

  block_t* curBlock = 0;

  block_t* emptyStack[BUFFER_BLOCK_COUNT];
  uint8_t emptyTop;
  uint8_t minTop;

  block_t* fullQueue[QUEUE_DIM];
  uint8_t fullHead = 0;
  uint8_t fullTail = 0;

  // Use SdFat's internal buffer.
  emptyStack[0] = (block_t*)sd.vol()->cacheClear();
  if (emptyStack[0] == 0) {
    error("cacheClear failed");
  }
  // Put rest of buffers on the empty stack.
  for (int i = 1; i < BUFFER_BLOCK_COUNT; i++) {
    emptyStack[i] = &block[i - 1];
  }
  emptyTop = BUFFER_BLOCK_COUNT;
  minTop = BUFFER_BLOCK_COUNT;

  // Start a multiple block write.
  if (!sd.card()->writeStart(binFile.firstBlock())) {
    error("writeStart failed");
  }
  console.print(F("FreeStack: "));
  console.println(FreeStack());
  console.println(F("Logging - type any character to stop"));
  bool closeFile = false;
  uint32_t bn = 0;
  uint32_t maxLatency = 0;
  uint32_t overrun = 0;
  uint32_t overrunTotal = 0;
  uint32_t logTime = micros();
  SetLogEndTime();

  while (1) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;
    if ((console.available()) || digitalRead(BOARD_BUTTON_PIN)) {
      StopLogging = true;
      closeFile = true;
    }
    if (closeFile) {
      if (curBlock != 0) {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
        curBlock = 0;
      }
    } else {
      if (curBlock == 0 && emptyTop != 0) {
        curBlock = emptyStack[--emptyTop];
        if (emptyTop < minTop) {
          minTop = emptyTop;
        }
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      if ((int32_t)(logTime - micros()) < 0) {
        error("Rate too fast");
      }


      //========== Log Data part ====================================================
      int32_t delta;
      do {
        delta = micros() - logTime;
      } while (delta < 0);

      if (curBlock == 0) {
        overrun++;
        overrunTotal++;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
#if ABORT_ON_OVERRUN
        console.println(F("Overrun abort"));
        break;
#endif  // ABORT_ON_OVERRUN       
      } else {
#if USE_SHARED_SPI
        sd.card()->spiStop();
#endif  // USE_SHARED_SPI   
        closeFile = !acquireData(&curBlock->data[curBlock->count++]);
        //========== Log Data part ====================================================
#if USE_SHARED_SPI
        sd.card()->spiStart();
#endif  // USE_SHARED_SPI      
        if (curBlock->count == DATA_DIM) {
          fullQueue[fullHead] = curBlock;
          fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
          curBlock = 0;
        }
      }
    }
    if (fullHead == fullTail) {
      // Exit loop if done.
      if (closeFile) {
        break;
      }
    } else if (!sd.card()->isBusy()) {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];
      fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
      // Write block to SD.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) {
        error("write data failed");
      }
      usec = micros() - usec;
      if (usec > maxLatency) {
        maxLatency = usec;
      }
      // Move block to empty queue.
      emptyStack[emptyTop++] = pBlock;
      bn++;
      if (bn == FILE_BLOCK_COUNT) {
        // File full so stop
        break;
      }
    }
  }
  if (!sd.card()->writeStop()) {
    error("writeStop failed");
  }
  console.print(F("Min Free buffers: "));
  console.println(minTop);
  console.print(F("Max block write usec: "));
  console.println(maxLatency);
  console.print(F("Overruns: "));
  console.println(overrunTotal);
  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) {
    console.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn)) {
      error("Can't truncate file");
    }
  }
}
//------------------------------------------------------------------------------
void recoverTmpFile() {
  uint16_t count;
  if (!binFile.open(TMP_FILE_NAME, O_RDWR)) {
    return;
  }
  if (binFile.read(&count, 2) != 2 || count != DATA_DIM) {
    error("Please delete existing " TMP_FILE_NAME);
  }
  console.println(F("\nRecovering data in tmp file " TMP_FILE_NAME));
  uint32_t bgnBlock = 0;
  uint32_t endBlock = binFile.fileSize() / 512 - 1;
  // find last used block.
  while (bgnBlock < endBlock) {
    uint32_t midBlock = (bgnBlock + endBlock + 1) / 2;
    binFile.seekSet(512 * midBlock);
    if (binFile.read(&count, 2) != 2) error("read");
    if (count == 0 || count > DATA_DIM) {
      endBlock = midBlock - 1;
    } else {
      bgnBlock = midBlock;
    }
  }
  // truncate after last used block.
  if (!binFile.truncate(512 * (bgnBlock + 1))) {
    error("Truncate " TMP_FILE_NAME " failed");
  }
  renameBinFile();
}
//-----------------------------------------------------------------------------
void renameBinFile() {
  while (sd.exists(binName)) {
    if (binName[BASE_NAME_SIZE + 1] != '9') {
      binName[BASE_NAME_SIZE + 1]++;
    } else {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9') {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }

  //  char tmpc[10];
  //  GetUnitID(tmpc);
  //  strncpy(binName, tmpc, 3);
  //
  //  dir_t d;
  //
  //  if (binFile.dirEntry(&d)) {
  //
  //  }
  //  else {
  //    error("binfile.dirEntry failed");
  //  }

  if (!binFile.rename(sd.vwd(), binName)) {
    error("Can't rename file");
  }
  console.print(F("File renamed: "));
  console.println(binName);
  console.print(F("File size: "));
  console.print(binFile.fileSize() / 512);
  console.println(F(" blocks"));
}

//------------------------------------------------------------------------------
void testSensor() {
  const uint32_t interval = 100000;
  int32_t diff;
  data_t data;
  console.println(F("\nTesting - type any character to stop\n"));
  // Wait for Serial Idle.
  delay(1000);
  printHeader(&console);
  uint32_t m = micros();
  while (!console.available()) {
    m += interval;
    do {
      diff = m - micros();
    } while (diff > 0);
    acquireData(&data);
    ledBlink(LED_GREEN, 50);
    delay(50);
    printData(&console, &data);
  }
}

//------------------------------------------------------------------------------

void setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
  console.begin(115200);

  // Wait for USB Serial
  /*  while (!console) {
      SysCall::yield();
    }
  */
  console.print(F("\nFreeStack: "));
  console.println(FreeStack());
  console.print(F("Records/block: "));
  console.println(DATA_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }
  // Allow userSetup access to SPI bus.
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  // Setup sensors.
  userSetup();

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
    sd.initErrorPrint(&console);
    fatalBlink();
  } else {
    delay(500);
    ledBlink(LED_GREEN, 1000);
    delay(500);
    ledBlink(LED_GREEN, 1000);
  }

  /*
       dateTimeCallback() sets the function
       that is called when a file is created
       or when a file's directory entry is
       modified by sync().

       The callback can be disabled by the call
       SdFile::dateTimeCallbackCancel()
  */
  SdFile::dateTimeCallback(FATdateTime);

  // recover existing tmp file.

  if (sd.exists(TMP_FILE_NAME)) {
    console.println(F("\nRecovering existing tmp file " TMP_FILE_NAME));
    recoverTmpFile();
  }

  //  if (sd.exists(TMP_FILE_NAME)) {
  //    console.println(F("\nType 'Y' to recover existing tmp file " TMP_FILE_NAME));
  //    while (!console.available()) {
  //      SysCall::yield();
  //    }
  //    if (console.read() == 'Y') {
  //      recoverTmpFile();
  //    } else {
  //      error("'Y' not typed, please manually delete " TMP_FILE_NAME);
  //    }
  //  }
}
//------------------------------------------------------------------------------
void loop(void) {
  // Read any Serial data.
  do {
    delay(10);
  } while (console.available() && console.read() >= 0);
  console.println();
  console.println(F("Main menu:"));
  console.println(F("1 - open existing bin file"));
  console.println(F("2 - convert file to csv"));
  console.println(F("3 - dump data to Serial"));
  console.println(F("4 - overrun error details"));
  console.println(F("5 - list files"));
  console.println(F("6 - record data"));
  console.println(F("7 - test without logging"));
  console.println(F("8 - GPS menu"));
  console.println(F("9 - RTC menu"));
  while (!console.available()) {
    if (!StopLogging)
      logData();
    SysCall::yield();
  }
#if WDT_YIELD_TIME_MICROS
  console.println(F("LowLatencyLogger can not run with watchdog timer"));
  SysCall::halt();
#endif

  char c = tolower(console.read());

  // Discard extra Serial data.

  do {
    delay(10);
  } while (console.available() && console.read() >= 0);

  if (ERROR_LED_PIN >= 0) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  if (c == '1') {
    openBinFile();
  } else if (c == '2') {
    binaryToCsv();
  } else if (c == '3') {
    dumpData();
  } else if (c == '4') {
    checkOverrun();
  } else if (c == '5') {
    console.println(F("\nls:"));
    sd.ls(&console, LS_SIZE);
  } else if (c == '6') {
    logData();
  } else if (c == '7') {
    testSensor();
  } else if (c == '8') {
    GPS_Menu();
  } else if (c == '9') {
    RTC_Menu();
  } else {
    console.println(F("Invalid entry"));
  }
}
