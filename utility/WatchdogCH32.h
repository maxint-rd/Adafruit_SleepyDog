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

// fix some CH32 core differences
#if defined(CH32X035)     // the CH32X035 define is also used for X033
    #define PWR_AWU_Prescaler_61440 AWU_Prescaler_61440
    #define PWR_AWU_Prescaler_10240 AWU_Prescaler_10240
    #define PWR_AWU_SetPrescaler AWU_SetPrescaler           // AWU_SetPrescaler only for X035/X033, PWR_AWU_SetPrescaler only for V003,VM00x
    #define PWR_AutoWakeUpCmd AutoWakeUpCmd // AutoWakeUpCmd only for X035/X033,  PWR_AutoWakeUpCmd only for V003,VM00x
    #define PWR_AWU_SetWindowValue AWU_SetWindowValue // AWU_SetWindowValue only for X035/X033,  PWR_AWU_SetWindowValue only for V003,VM00x
    #define PWR_STANDBYEntry_WFE    // no different standby entries for CH32X035/X033: void PWR_EnterSTANDBYMode(void)
#endif

/**************************************************************************/
/*!
    @brief  Class that contains functions for interacting with the
            CH32's hardware watchdog timer
*/
/**************************************************************************/
class WatchdogCH32 {
public:
  //WatchdogCH32() : _wdto(-1){};  // constuctor init private vars
  int enable(int maxPeriodMS = 0);  // enable watchdog
  void disable()      __attribute__((error("CH32 IWDG cannot be disabled!")));
  void reset();   // feed the dog
#if defined(CH32V00x) || defined(CH32X035)
  int sleep(int maxSleepMS = 30000);
#else
  int sleep(int maxSleepMS = 0) __attribute__((error("CH32 sleep() only implemented for CH32V00x/X033/X035! (for now?)")));
#endif

private:
  int _wdto;
  void iwdg_setup(uint16_t reload_val, uint8_t prescaler);
  void iwdg_feed();
  uint32_t pwr_sleep(uint32_t uSleep);
  void gpios_on();
  void gpios_off();
  void EXTI_INT_INIT();



};

#endif // WATCHDOGCH32_H_