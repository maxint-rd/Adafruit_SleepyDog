#if defined(ARDUINO_ARCH_CH32) || defined(CH32V00x) || defined(CH32X035)

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

/* was used in CH32X035 example
void WatchdogCH32::EXTI_INT_INIT(void)
{   // copied from https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PWR/Standby_Mode/User/main.c
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    EXTI_InitStructure.EXTI_Line = EXTI_Line27;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}
*/

extern "C" {
#if defined(CH32VM00X) || defined(CH32V00x)
extern __IO uint32_t msTick;      // the msTick counter will be updated after sleeping
#else
extern __IO uint64_t msTick;      // the msTick counter will be updated after sleeping
#endif
}


#if defined(CH32X035)
#define OPT_X035_USE_INTERRUPT 0
#if (OPT_X035_USE_INTERRUPT)
extern "C" {
void AWU_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void AWU_IRQHandler()
{
    EXTI_ClearITPendingBit(EXTI_Line27);
    SystemInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);  // SDI=system debug interface
    //Serial.printf("\r\n Auto wake up \r\n");
}
} // extern "C"
#endif // #if defined(OPT_X035_USE_INTERRUPT)
#endif // #if defined(CH32X035)

uint32_t WatchdogCH32::pwr_sleep(uint32_t uSleepMS)
{ // Sleep for some time, then wake up automatically
  // The CH32V003 has a Low Speed clock (running at 128kHz), that can be used to wakeup from standby.
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
      
   CH32X035/X033 has no LSI clock. To wakeup 47KHz divided clock of the internal high-speed clock HSI as the AWU module clock source:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
   To wake up from low power mode, the external interrupt line 27 needs to be configured as a rising edge interrupt.
   See also AWU in the EVT Standby example: https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/PWR/Standby_Mode/User/main.c:
      AWU_SetPrescaler(AWU_Prescaler_10240);
      AWU_SetWindowValue(25);  // 25/(48M/1024/10240) = 5.46s
      AutoWakeUpCmd(ENABLE);
      PWR_EnterSTANDBYMode();
*/

  // calculate sleep settings and repeat until all sleep is done
  uint32_t nSleepLeft=uSleepMS;
  uint32_t nSleepDone=0;
  //uint32_t nSleep=128000/61440;   // 128000/61440=2.08333, 63/128000/61440=30,240
  uint32_t uPrescaler=PWR_AWU_Prescaler_61440;
  uint32_t uWindow=63;
//Serial.print("z");
  
  // Nap repeatedly, until indicated sleep time has passed
  while(nSleepLeft>500)   // PWR_AWU_Prescaler_61440 allows for 0.48s resolution on V003
  {
//Serial.print("0");
    // Determine prescaler and window values needed for largest nap we can do
    // TODO: See if other prescaler provides better fit. For short durations a smaller prescaler is more precise.
    // V003: Highest prescaler PWR_AWU_Prescaler_61440 allows for 0.48 up to 30.24 seconds sleep
    //       Prescaler_10240 offers finer resolution for 0.08 up to 5.04 seconds sleep
    //       128000/61440=2.08333, 63/128000/61440=30.240
    //       128000/10240=12.5, 63/128000/10240=5.040
    // X035: HSI 48MHz clock is used with a divider of 1024, giving a 46.875kHz divided clock.
    //       Highest prescaler AWU_Prescaler_61440 gives longggg sleep (82,6 sec) at terrible resolution
    //       Prescaler_10240 offers finer resolution for 0.22 up to 13.76 seconds sleep
    //       48000000/1024/61440=0,762939453125 => 1,31072 sec/count (max 82,57536 sec)
    //       48000000/1024/10240=4,57763671875 => 0,21845333 sec/count (max 13,76256 sec)
#if defined(CH32V00x) || defined(CH32VM00X)
    #define AWU_DIVIDED_COUNTS_PER_KSEC 2083   // 128000/61440=2.08333 => 0.48 sec/count
#elif defined(CH32X035)
    uPrescaler=PWR_AWU_Prescaler_10240;
    #define AWU_DIVIDED_COUNTS_PER_KSEC 4578  // 48000000/1024/10240=4,57763671875 => 0,21845333 sec/count (max 13,76256 sec)
#endif

    // find the window value that best matches sleep time left
    uWindow=63;  // IWDG window downcounter must be lower than 0x3F=63d
    while((uWindow*1000000L)/AWU_DIVIDED_COUNTS_PER_KSEC>nSleepLeft && uWindow>1)
      uWindow--;

    // calculate actual sleep time for window value
    uint32_t nSleep=(uWindow*1000000L)/AWU_DIVIDED_COUNTS_PER_KSEC;
    nSleepDone+=nSleep;
    nSleepLeft-=nSleep;

    // enable power interface module clock
#if defined(CH32V00x) || defined(CH32X035)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
#elif defined(CH32VM00X)
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR, ENABLE);
#else
    //#warning(NOTE: no support for other CH32 mcu) 
#endif


    // TODO: I saw no function that can be called to enable the AutoWakeUp event. WCH Examples use EXTI_Init().
    // Only CH32V003 uses EXTI_Line9 for the wake up event. Other family members use other lines (eg. CH32X035 uses EXTI_Line27)
    // Unfortunately I don't have all other CH32 chips to test and add support.
    // As far as I can see the current core (1.0.4) has no common function to select the line used for AutoWakeUp.
    // I suggest to make this all part of some function related to PWR_AutoWakeUpCmd()
    // V003/VM00X: enable AutoWakeUp event  EXTI_Init( EXTI_Line9 EXTI_Mode_Event EXTI_Trigger_Falling ENABLE)
    // X035: enable AutoWakeUp event  EXTI_Init( EXTI_Line27 EXTI_Mode_Interrupt EXTI_Trigger_Falling ENABLE)
#if defined(CH32V00x) || defined(CH32VM00X)
    EXTI->EVENR |= EXTI_Line9;
    EXTI->FTENR |= EXTI_Line9;
#elif defined(CH32X035)
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE); // when allowing pin wakeup

    //EXTI->EVENR |= EXTI_Line27;
    EXTI->INTENR |= EXTI_Line27;      // no event, only intterrupt
    EXTI->FTENR |= EXTI_Line27;
    //EXTI_INT_INIT();

  #if (OPT_X035_USE_INTERRUPT)
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = AWU_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  #endif
#else
    //#warning(NOTE: no support for other CH32 mcu) 
#endif

    // configure AWU prescaler
    PWR_AWU_SetPrescaler(uPrescaler); // possible values: 1/2/4/8/.../4096, 10240, 61440

    // configure AWU window comparison value
    PWR_AWU_SetWindowValue(uWindow);

#if defined(CH32V00x) || defined(CH32VM00X)
    // enable low speed oscillator (LSI)
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
#endif

    // enable AWU
    PWR_AutoWakeUpCmd(ENABLE);

    // Select deep sleep on power-down (PWR_CTLR_PDDS) and use WFE command to enter Sleep mode.
    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);       // PWR_STANDBYEntry_WFI is wake up by interrupt, _WFE is wake up by event, 
                                                      // _WFE only available on V003/VM00x, X035 always calls __WFI()=Wait for Interrupt
#if defined(CH32V00x) || defined(CH32VM00X)
    // Back from sleep; restore clock to full speed
    SystemInit();
#elif defined(CH32X035)
#if !(OPT_X035_USE_INTERRUPT)
    EXTI_ClearITPendingBit(EXTI_Line27);  // clear the pending bit so we can sleep again later
    SystemInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);  // SDI=system debug interface
#endif
#endif
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
  //_wdto = maxPeriodMS;

  // Enables the CH32's hardware WDT with maxPeriodMS delay
  // (wdt should be updated every maxPeriodMS ms) and
  // enables pausing the WDT on debugging when stepping thru
  //watchdog_enable(maxPeriodMS, 1);
  
  // based on CH32V003 datasheet and on IWDG example from ch32v003fun by CNLOHR:
  //   https://github.com/cnlohr/ch32v003fun/blob/master/examples/iwdg/iwdg.c
  
  // TODO: find best values for prescaler and counter to match maxPeriodMS
  //       Return value actually used (in ms)

  // On the CH32V003 the independent watchdog uses the LSI clock which runs at 128kHz, using the IWDG_Prescaler_128, the 12 bit counter allows up to 4096 msec
  // On the CH32X035/X033, the 48000MHz HSI clock is used with a 1024 divider. 48MHz/1024=46.875kHz. Using (IWDG_Prescaler_32, 4000) gives 2.7s IWDG reset
	// set up watchdog (0xfff=4096d, with prescaler 128 this is about 4sec on the CH32V003)
#if defined(CH32V00x) || defined(CH32VM00X)
  uint8_t prescaler=IWDG_Prescaler_128;
  #define PERIOD_FIX(x) (x)
  #define PERIOD_FIX_REVERSE(x) (x)
#endif
#if defined(CH32X035)
  uint8_t prescaler=IWDG_Prescaler_32;
  // transform maxPeriodMS to comply to 46.875kHz clock and smaller prescaler. 
  //#define PERIOD_FIX(x)  ((uint16_t)(((uint32_t)(x)*4L*46875L)/128000L))    // 46875L/128000L=0,3662109375
  //#define PERIOD_FIX_REVERSE(x)  ((uint16_t)(((uint32_t)(x)*128000L)/(4L*46875L)))    // 128000L/46875L=2.73066667
  #define PERIOD_FIX(x)  ((uint16_t)(((uint32_t)(x)*1875L)/1280L))    // 4L*46875L=187500L, 46875L/128000L=0,3662109375
  #define PERIOD_FIX_REVERSE(x)  ((uint16_t)(((uint32_t)(x)*1280L)/1875L))    // 4L*46875L=187500L,  128000L/46875L=2.73066667
  //Serial.printf("\nMAXP:%d\n", PERIOD_FIX(maxPeriodMS));
#endif
  if(PERIOD_FIX(maxPeriodMS)<=0xfff)    // < 4096
    iwdg_setup(PERIOD_FIX(maxPeriodMS), prescaler);  // set up watchdog
  else
  {   // CH32 supports prescaler up to 256, allowing for max 8192 mSec timout on V003. Since a different clock is used that duration may be not very precise
      // On the X035/X035 the HSI is used with a 1024 devider. Using the 256 prescaler gives a maximum timeout of 22.3 sec.
#if defined(CH32V00x) || defined(CH32VM00X)
    prescaler=IWDG_Prescaler_256;
    if(maxPeriodMS>0x1fff)
      maxPeriodMS=0x1fff;
    iwdg_setup(maxPeriodMS/2, prescaler);  // set up watchdog
#endif
#if defined(CH32X035)
    prescaler=IWDG_Prescaler_256; // use largest prescaler
    //if((uint16_t)(PERIOD_FIX(maxPeriodMS)/8L)>(uint16_t)0xfff)
    //  maxPeriodMS=(uint16_t)(PERIOD_FIX_REVERSE((uint16_t)0xfff*8L));
    if(maxPeriodMS>22364)   // maximum 22364ms found using PERIOD_FIX_REVERSE
      maxPeriodMS=22364;
    iwdg_setup(PERIOD_FIX(maxPeriodMS)/8L, prescaler);  // using largest prescaler still requires reload_val to be <0x1000 (4096d)
#endif
//    if(PERIOD_FIX(maxPeriodMS)>0x1fff)
//      maxPeriodMS=PERIOD_FIX_REVERSE(0x1fff);
//    iwdg_setup(PERIOD_FIX(maxPeriodMS/2), prescaler);  // set up watchdog
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
#if defined(CH32V00x)
  #warning(NOTE: sleep on CH32V003 only works after power cycle) 
#endif
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
