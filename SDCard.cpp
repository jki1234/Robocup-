#include "SDCard.h"

#include "MapParser.h"
#include "Utils.h"

// See:
// https://docs.arduino.cc/learn/programming/sd-guide/#read-and-write

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <stdarg.h>

static File logFile;

static volatile bool logReady = false;
static unsigned long lastFlushMicros = 0;

// Buffer up the I/O to avoid wearing out the SD card.
static char sdBuf[1024 * 8]; // This is NOT null-terminated!
static volatile uint32_t sdBufPos = 0;
static char lineBuf[128]; // Sets the max line length, including the newline

static uint8_t mapZipData[64 << 10]; // Support zip files upto 64k

void sdInit() {
  Serial.print("Initializing SD card...");

  // See https://www.pjrc.com/store/teensy40.html
  // Originally I used BUILTIN_SDCARD and that didn't work for whatever reason?
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  File root = SD.open("/");
  while (true) {
    File f = root.openNextFile();

    // Check if it ends in .zip
    const char *name = f.name();
    int nameLen = strlen(name);
    const char *extension = ".robozip";
    int extLen = strlen(extension);
    if (nameLen <= extLen || strcmp(name + nameLen - extLen, extension) != 0) {
      f.close();
      continue;
    }

    serialPrintf("Attempting to load file as map: %s", name);

    int length = (int)f.size();
    if (length >= (int)sizeof(mapZipData)) {
      serialPrintf("File is too big! Can't fit in buffer.");
      f.close();
      continue;
    }

    int readCount = f.read(mapZipData, length);
    if (readCount != length) {
      serialPrintf("Failed to read map data - only read %d bytes of %d.", readCount, length);
      f.close();
      continue;
    }

    // We've read the file, we're done with it now.
    f.close();

    // Attempt to load the map
    MapLoadErrno err = mapLoad(mapZipData, length);
    if (err != 0) {
      serialPrintf("Failed to process map data: error %d", err);
      continue;
    }

    serialPrintf("Loaded map data!");
    break;
  }
  root.close();
}

void loggerInit() {
  // Create a file with a new, unique name.
  // Without this, we'd overwrite whatever file we'd previously written.
  char filename[16];
  for (int i = 0; true; i++) {
    snprintf(filename, sizeof(filename), "log-%04d.csv", i);
    if (SD.exists(filename)) {
      continue;
    }
    logFile = SD.open(filename, FILE_WRITE);
    break;
  }

  if (!logFile) {
    Serial.println("Failed to open log file!");
    while (true)
      ;
  }

  logReady = true;
}

void loggerFlush() {
  Serial.println("Flushing SD card");
  logFile.write(sdBuf, sdBufPos);
  logFile.flush();
  sdBufPos = 0;
  lastFlushMicros = micros();
}

void loggerPeriodicFlush(unsigned long millisecondTime) {
  // Is there anything to flush?
  if (sdBufPos == 0) {
    return;
  }

  // Flush if the ISR is filling the buffer up, otherwise that won't flush except on the timer.
  size_t remaining = sizeof(sdBuf) - sdBufPos;
  size_t margin = sizeof(sdBuf) / 5;

  if (lastFlushMicros == 0) {
    lastFlushMicros = micros();
  }
  unsigned long elapsed = micros() - lastFlushMicros;
  if (elapsed / 1'000 > millisecondTime || remaining < margin) {
    loggerFlush();
  }
}

void loggerWrite(const char *msg, ...) {
  // First, format the message
  // See https://linux.die.net/man/3/snprintf
  // (I'm assuming the Arduino libc matches this)
  va_list list;
  va_start(list, msg);
  size_t n = vsnprintf(lineBuf, sizeof(lineBuf), msg, list);
  va_end(list);

  // If the message was truncated, put in a warning at the end of it
  if (n >= sizeof(lineBuf)) {
    const char *warning = "<TRUNC>\n";
    int len = (int)strlen(warning) + 1; // +1 for null byte
    strcpy(lineBuf + sizeof(lineBuf) - len, warning);
    n = sizeof(lineBuf) - 1; // Don't include the null byte
  }

  if (sizeof(sdBuf) - sdBufPos <= n + 1) {
    loggerFlush();
  }

  memcpy(sdBuf + sdBufPos, lineBuf, n);
  sdBufPos += n;

  sdBuf[sdBufPos++] = '\n';
}

void loggerWriteISR(const char *msg, ...) {
  // This is racy with loggerFlush.
  // I don't care, as this is only used for debugging. If we loose a bit of data, that's just tough.

  if (!logReady)
    return;

  // First, format the message
  // See https://linux.die.net/man/3/snprintf
  // (I'm assuming the Arduino libc matches this)
  va_list list;
  va_start(list, msg);
  size_t n = vsnprintf(lineBuf, sizeof(lineBuf), msg, list);
  va_end(list);

  // If the message was truncated, put in a warning at the end of it
  if (n >= sizeof(lineBuf)) {
    const char *warning = "<TRUNC>\n";
    int len = (int)strlen(warning) + 1; // +1 for null byte
    strcpy(lineBuf + sizeof(lineBuf) - len, warning);
    n = sizeof(lineBuf) - 1; // Don't include the null byte
  }

  // Can't flush since we're in an ISR
  n = min(n, sizeof(sdBuf) - sdBufPos);

  memcpy(sdBuf + sdBufPos, lineBuf, n);
  sdBufPos += n;
}
