// librosco - a communications library for the Rover MEMS ECU
//
// timing.c: This file contains for handling delays and timing
//           of communications.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#if defined(WIN32)
  #include <windows.h>
#elif defined(__NetBSD__)
  #include <string.h>
#endif

#include "rosco.h"
#include "rosco_internal.h"

/**
 * Get the current microseconds.
 */
uint32_t get_current_us()
{
  static LARGE_INTEGER frequency;
  LARGE_INTEGER current_time;

  // Only get the frequency once
  if (frequency.QuadPart == 0)
  {
    QueryPerformanceFrequency(&frequency);
  }

  // Get the current time
  QueryPerformanceCounter(&current_time);
  return (current_time.QuadPart * 1000000) / frequency.QuadPart;
}

/**
 * Wait until the current microseconds equals a certain value...
 */
uint32_t wait_until_us(uint32_t us)
{
  uint32_t current_us;

  do
  {
    current_us = get_current_us();
  } while (current_us < us);

  return current_us;
}

/**
 * Wait for a number of microseconds.
 */
void wait_for_us(uint32_t us)
{
  wait_until_us(get_current_us() + us);
}