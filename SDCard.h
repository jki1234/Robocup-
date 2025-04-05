#pragma once

/**
 * Initialise the SD card.
 *
 * This scans for and loads the first map file it finds.
 */
void sdInit();

/**
 * Initialise the logging library.
 *
 * This creates a new file on the SD card.
 */
void loggerInit();

/**
 * Make sure all the data we've "written" so far is actually
 * saved on the SD card.
 */
void loggerFlush();

/**
 * Flush the logger if it's been more than a given number of milliseconds
 * since we last flushed the SD card.
 */
void loggerPeriodicFlush(unsigned long millisecondTime);

/**
 * Write a string to the SD card. This uses the printf syntax.
 *
 * Don't forget to add a newline!
 *
 * Note this doesn't immediately write the data to the SD card, as doing
 * that too frequently can wear out the card's flash memory. Instead, it's
 * stored in a buffer in RAM until you call loggerFlush().
 */
void loggerWrite(const char *msg, ...);

/**
 * Like loggerWrite, but suitable for use in an ISR.
 *
 * This is a little sketchy, so only use it for debugging.
 *
 * This may loose some data if the file isn't being flushed often enough.
 */
void loggerWriteISR(const char *msg, ...);
