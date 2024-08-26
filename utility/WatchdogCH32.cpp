#if defined(ARDUINO_ARCH_CH32)

#include "WatchdogCH32.h"

/**********************************************************************************************/
/*!
    @brief  Initializes the CH32's hardware watchdog timer.
    @param    maxPeriodMS
              Timeout period of WDT in milliseconds
    @return The actual period (in milliseconds) before a watchdog timer
            reset is returned, 0 otherwise.
*/
/**********************************************************************************************/

void WatchdogCH32::iwdg_setup(uint16_t reload_val, uint8_t prescaler) {
  // See https://github.com/openwch/ch32v003/blob/main/EVT/EXAM/IWDG/IWDG/User/main.c
  //IWDG_Disable();      // Disable IWDG
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // Enable access to IWDG_PSCR and IWDG_RLDR registers.
  IWDG_SetPrescaler(prescaler);   // Set IWDG Prescaler value.
  IWDG_SetReload(reload_val & 0xfff);     // specify the IWDG Reload value. This parameter is a 12-bit value ao it must be a number between 0 and 0x0FFF.
  IWDG_ReloadCounter();     // ensure reload value is changed when iwdg_setup() is called again, but with different parameters
  IWDG_Enable();      // Enable IWDG (write access to IWDG_PSCR and IWDG_RLDR registers disabled).
}

void WatchdogCH32::iwdg_feed() {
  IWDG_ReloadCounter();
}

/*
void WatchdogCH32::gpios_on() {
	GPIOD->BSHR = 1 | (1<<4);
	GPIOC->BSHR = 1;
}

void WatchdogCH32::gpios_off() {
	GPIOD->BSHR = (1<<16) | (1<<(16+4));
	GPIOC->BSHR = (1<<16);
}
*/

extern "C" {
extern __IO uint64_t msTick;      // the msTick counter will be updated after sleeping
}

uint32_t WatchdogCH32::pwr_sleep(uint32_t uSleepMS)
{ // Sleep for some time, then wake up automatically
  // The longest single sleep amount is approx 30 seconds (as tested on CH32V003). 
  // To reach the requested amount of sleep, repeated naps are done until the requested amount is reached
  // WARNING: You MUST power cycle the CH32V003 to allow it to go into deep sleep. After reset by flashing or by Watchdog sleep is NOY enabled!

/* Partially based on ch32v003 fun example: https://github.com/cnlohr/ch32v003fun/blob/master/examples/standby_autowake/standby_autowake.c
   This example serves to show how to put the CH32V003 into its lowest power state (standby) and have it wake periodically.
   Power consumption could be around 10uA, when all GPIOs are set to input pull up
   The autowakeup delay can be calculated by: t = AWUWR / (fLSI / AWUPSC), where 
      - AWUWR can be 1 to 63, 
      - fLSI is always 128000 
      - AWUPSC - for practical purposes - is 2048, 4096, 10240 or 61440, though lower values are possible.
   The maximum autowakeup delay is 30s.
   See also this discussion about AWU power cycle: https://github.com/cnlohr/ch32v003fun/issues/233

   See also AWU in the EVT Standby example: /EVT/EXAM/PWR/Standby_Mode/User/main.c:
      RCC_LSICmd(ENABLE);
      while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
      PWR_AWU_SetPrescaler(PWR_AWU_Prescaler_10240);
      PWR_AWU_SetWindowValue(25);
      PWR_AutoWakeUpCmd(ENABLE);
      PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);
*/

  // calculate sleep settings and repeat until all sleep is done
  uint32_t nSleepLeft=uSleepMS;
  uint32_t nSleepDone=0;
  //uint32_t nSleep=128000/61440;   // 128000/61440=2.08333, 63/128000/61440=30,240
  uint32_t uPrescaler=PWR_AWU_Prescaler_61440;
  uint32_t uWindow=63;
  
  // Nap repeatedly, until indicated sleep time has passed
  while(nSleepLeft>500)   // PWR_AWU_Prescaler_61440 allows for 0.48s resolution
  {
    // Determine prescaler and window values needed for largest nap we can do
    // TODO: See if other prescaler provides better fit. For short durations a smaller prescaler is more precise.
    // Highest prescaler PWR_AWU_Prescaler_61440 allows for 0.48-30.24 seconds sleep
    // Prescaler_10240 offers finer resolution for up to 0.08-5.04 seconds sleep
    // 128000/61440=2.08333, 63/128000/61440=30.240
    // 128000/10240=12.5, 63/128000/10240=5.040

    // find the window value that best matches sleep time left
    uWindow=63;
    while((uWindow*1000000L)/2083>nSleepLeft && uWindow>1)
      uWindow--;

    // calculate actual sleep time for window value
    uint32_t nSleep=(uWindow*1000000L)/2083;
    nSleepDone+=nSleep;
    nSleepLeft-=nSleep;

    // enable power interface module clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    // TODO: I saw no function that can be called to enable the AutoWakeUp event. Perhaps it is EXTI_Init()?
    // Only CH32V003 uses EXTI_Line9 for the wake up event. Other family members use other lines (eg. CH32X035 uses EXTI_Line27)
    // Unfortunately I don't have other CH32 chips to test and add support.
    // As far as I can see the current core (1.0.4) has no common function to select the line used for AutoWakeUp.
    // I suggest to make this all part of some function related to PWR_AutoWakeUpCmd()
    // enable AutoWakeUp event  EXTI_Init( EXTI_Line9 EXTI_Mode_Event EXTI_Trigger_Falling ENABLE)
    EXTI->EVENR |= EXTI_Line9;
    EXTI->FTENR |= EXTI_Line9;

    // configure AWU prescaler
    PWR_AWU_SetPrescaler(uPrescaler); // possible values: 1/2/4/8/.../4096, 10240, 61440

    // configure AWU window comparison value
    PWR_AWU_SetWindowValue(uWindow);

    // enable low speed oscillator (LSI)
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    // enable AWU
    PWR_AutoWakeUpCmd(ENABLE);

    // Select deep sleep on power-down (PWR_CTLR_PDDS) and use WFE command to enter Sleep mode.
    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);       // PWR_STANDBYEntry_WFI is wake up by interrupt, _WFE is wake up by event
    
    // Back from sleep; restore clock to full speed
    SystemInit();
  }
  // Update millis() with approximation of time spend sleeping...
  msTick+=nSleepDone;     // TODO: More accurate measument of sleep done. Perhaps we could use millis() to measure time for calls above
  return(nSleepDone);
}


/*
**
**     Public methods
**
*/


int WatchdogCH32::enable(int maxPeriodMS) {
  if (maxPeriodMS < 0)
    return 0;

  // Enables the CH32's hardware WDT with maxPeriodMS delay
  // (wdt should be updated every maxPeriodMS ms) and
  // enables pausing the WDT on debugging when stepping thru
  //watchdog_enable(maxPeriodMS, 1);
  
  // based on CH32V003 datasheet and on IWDG example from ch32v003fun by CNLOHR:
  //   https://github.com/cnlohr/ch32v003fun/blob/master/examples/iwdg/iwdg.c
  
  // TODO: find best values for prescaler and counter to match maxPeriodMS
  //       Return value actually used (in ms)

  // The independent watchdog uses the LSI clock which runs at 128kHz, using the IWDG_Prescaler_128, the 12 bit counter allows up to 4096 msec
	// set up watchdog (0xfff=4096d, with prescaler 128 this is about 4sec on the CH32V003)
  uint8_t prescaler=IWDG_Prescaler_128;
  if(maxPeriodMS<=0xfff)
    iwdg_setup(maxPeriodMS, prescaler);  // set up watchdog
  else
  {   // CH32 supports prescaler up to 256, allowing for max 8192 mSec timout. Since a different clock is used that duration may be not very precise
    prescaler=IWDG_Prescaler_256;
    if(maxPeriodMS>0x1fff)
      maxPeriodMS=0x1fff;
    iwdg_setup(maxPeriodMS/2, prescaler);  // set up watchdog
  }

 /*
 * @param   IWDG_Prescaler - specifies the IWDG Prescaler value.
 *             IWDG_Prescaler_4 - IWDG prescaler set to 4.
 *             IWDG_Prescaler_8 - IWDG prescaler set to 8.
 *             IWDG_Prescaler_16 - IWDG prescaler set to 16.
 *             IWDG_Prescaler_32 - IWDG prescaler set to 32.
 *             IWDG_Prescaler_64 - IWDG prescaler set to 64.
 *             IWDG_Prescaler_128 - IWDG prescaler set to 128.
 *             IWDG_Prescaler_256 - IWDG prescaler set to 256.
 *
 */
 
  _wdto = maxPeriodMS;
  return maxPeriodMS;
}

/**************************************************************************/
/*!
    @brief  Reload the watchdog counter with the amount of time set in
            enable().
*/
/**************************************************************************/
void WatchdogCH32::reset() { 
  iwdg_feed();
}

/**************************************************************************/
/*!
    @brief  Once enabled, the CH32's Independent Watchdog can NOT be disabled.
*/
/**************************************************************************/
void WatchdogCH32::disable() {
}

/**************************************************************************/
/*!
    @brief  Configures the CH32 to enter a lower power (WFE) sleep
            for a period of time.
    @param    maxSleepMS
              Time to sleep the CH32, in millis.
    @return The actual period (in milliseconds) that the hardware was
            asleep will be returned. Otherwise, 0 will be returned if the
            hardware could not enter the low-power mode.
*/
/**************************************************************************/
int WatchdogCH32::sleep(int maxSleepMS) {
  #warning(NOTE: sleep only works after power cycle) 
  if (maxSleepMS < 0)
    return 0;
  maxSleepMS=pwr_sleep(maxSleepMS);

  // CH32V003 has these low power modes:
  //  - Sleep mode: The core stops running and all peripherals (including core private peripherals) are still running.
  //  = Standby mode: Stop all clocks, wake up and switch the clock to HSI.  

  // perform a lower power (WFE) sleep (pico-core calls sleep_ms(sleepTime))
  //__attribute__((error("CH32 WDT sleep() not implemented yet!")));
  //sleep_ms(maxPeriodMS);

/*
  // TODO: convert requested sleep time to required counter value and prescaler
	standby_autowakeup_init(63, PWR_AWU_Prescaler_10240);

	standby_gpio_init();
	standby_gpio_assign_button(GPIOD, 2);
	standby_gpio_assign_pin(STANDBY_GPIO_PORT_D, 2, STANDBY_TRIGGER_DIRECTION_FALLING);

	standby_enter();

*/

  return maxSleepMS;
}

#endif // ARDUINO_ARCH_CH32
