#include "Utils.h"

#include <Arduino.h>
#include <stdio.h>

static char serialPrintBuf[256];

void serialInit() {
  Serial.begin(115200);
}

void serialPrintf(const char *format, ...) {
  // Run printf on the varargs argument
  va_list args;
  va_start(args, format);
  vsnprintf(serialPrintBuf, sizeof(serialPrintBuf), format, args);
  va_end(args);

  // Print the resulting message
  Serial.println(serialPrintBuf);
}
