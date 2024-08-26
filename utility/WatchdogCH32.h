/*!
 * @file WatchdogCH32.h
 *
 * Support for CH32 IWDG Hardware Watchdog Timer API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Structure based on WatchdogRP2040 by Brent Rubell for Adafruit Industries.
 * Support for CH32V00x by Maxint R&D
 *
 * MIT License, all text here must be included in any redistribution.
 *
 */
#ifndef WATCHDOGCH32_H_
#define WATCHDOGCH32_H_

#include "Arduino.h"

//#include <hardware/watchdog.h>

/**************************************************************************/
/*!
    @brief  Class that contains functions for interacting with the
            CH32's hardware watchdog timer
*/
/**************************************************************************/
class WatchdogCH32 {
public:
  //WatchdogCH32() : _wdto(-1){};
  int enable(int maxPeriodMS = 0);  // enable watchdog
  void disable()      __attribute__((error("CH32 IWDG cannot be disabled!")));
  void reset();   // feed the dog
#if defined(CH32V00x)
  int sleep(int maxSleepMS = 30000);
#else
  int sleep(int maxSleepMS = 0) __attribute__((error("CH32 sleep() only implemented for CH32V00x! (for now?)")));
#endif

private:
  int _wdto;
  void iwdg_setup(uint16_t reload_val, uint8_t prescaler);
  void iwdg_feed();
  uint32_t pwr_sleep(uint32_t uSleep);
  void gpios_on();
  void gpios_off();



};

#endif // WATCHDOGCH32_H_