#pragma once

#ifdef DEBUG
#define DEBUG_BOOL 0
#else
#define DEBUG_BOOL 0
#endif

/**
 * If the DEBUG macro is defined, print a printf-formatted string to the serial port.
 *
 * This appends a newline automatically.
 *
 * If not, do nothing.
 */
#define DEBUG_PRINTF(...)                                                                                                                                                                              \
  do {                                                                                                                                                                                                 \
    if (DEBUG_BOOL)                                                                                                                                                                                    \
      serialPrintf(__VA_ARGS__);                                                                                                                                                                       \
  } while (0)

/**
 * Get the number of items in an array.
 *
 * This has the same name/behaviour as the Win32 macro of the same name.
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

void serialInit();

// Note the __attribute__ makes GCC show warnings if you use the wrong argument types.
// https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html
void serialPrintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
