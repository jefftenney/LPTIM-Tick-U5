// Ultra Low Power API implementation (ulp.c)

#include "stm32u5xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ulp.h"

#define SUPPORT_VREG_RANGES_1_THROUGH_3

static uint32_t ramKey  __attribute__ ((section (".noinit")));
static uint32_t minTime __attribute__ ((section (".noinit")));
static uint32_t maxTime __attribute__ ((section (".noinit")));
void vUlpInit()
{
   //      Turn on the peripheral clock to the PWR module.  We use that module to control low-power options.
   //
   __HAL_RCC_PWR_CLK_ENABLE();

   //      Use the brownout reset supervisor in its lowest-power mode.
   //
   PWR->CR1 |= PWR_CR1_ULPMEN;

   //      Select the DC-DC SMPS instead of the LDO.  The SMPS gives us the lowest power consumption.
   //
   PWR->CR3 |= PWR_CR3_REGSEL;

   //      Be sure the MCU wakes up from stop mode on the same clock we normally use as the core clock, if
   // possible.  Might as well give the MCU a head start getting the clock going while waking from STOP.
   //
   #define RCC_CFGR1_SW_HSI16  RCC_CFGR1_SW_0
   #define RCC_CFGR1_SW_MSIS   0
   #define RCC_CFGR1_SW_PLL   (RCC_CFGR1_SW_0 | RCC_CFGR1_SW_1)
   if ( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_HSI16 )
   {
      SET_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
   }
   else if ( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_MSIS )
   {
      CLEAR_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
   }
   else if ( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_PLL )
   {
      //      The application has selected the PLL (PLL1) as the system clock.  In this case, we optimize the
      // wake up timing by configuring the MCU to wake from the source clock for the PLL (MSIS or HSI16).
      //
      #define RCC_PLL1CFGR_PLL1SRC_MSIS   RCC_PLL1CFGR_PLL1SRC_0
      #define RCC_PLL1CFGR_PLL1SRC_HSI16  RCC_PLL1CFGR_PLL1SRC_1
      if ( (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC_Msk) == RCC_PLL1CFGR_PLL1SRC_HSI16 )
      {
         SET_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
      }
      else if ( (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC_Msk) == RCC_PLL1CFGR_PLL1SRC_MSIS )
      {
         CLEAR_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
      }
      else
      {
         //      The clock source for the PLL is HSE, but the MCU won't wake from stop mode on HSE.  So don't
         // change which clock helps us wake from stop mode -- MSIS or HSI16.
         //
         //      Note that the source for the core-voltage booster is also HSE in this case.  We don't support
         // HSE as the clock source to the core-voltage booster, unless the HSE is in bypass mode.
         //
         configASSERT( !(PWR->VOSR & PWR_VOSR_BOOSTEN) || (RCC->CR & RCC_CR_HSEBYP) );
      }
   }

   //      Enable the cycle counter in the Data Watchpoint and Trace unit.  A debugger enables this unit for
   // us, but when we're not running on a debugger, we have to enable it ourselves.  First we enable the DWT
   // (along with the PMU and ITM), and then we tell the DWT to turn on the cycle counter.  The ARMv8-M
   // Architecture Reference Manual describes the core's debug and trace components including the DWT system
   // (section B14).  Register detail is found in section D1.2.
   //
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

   // diagnostics: reset min/max values if lost during reset
   #define RAM_KEY_VALUE  0x3EFFA10E
   if (ramKey != RAM_KEY_VALUE)
   {
      minTime = UINT32_MAX;
      maxTime = 0;
      ramKey = RAM_KEY_VALUE;
   }
}

static volatile int xDeepSleepForbiddenFlags = 0;

void vUlpOnPeripheralsActive( int xPeripherals )
{
   taskENTER_CRITICAL();
   xDeepSleepForbiddenFlags |= xPeripherals;
   taskEXIT_CRITICAL();
}

void vUlpOnPeripheralsActiveFromISR( int xPeripherals )
{
   UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
   xDeepSleepForbiddenFlags |= xPeripherals;
   taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void vUlpOnPeripheralsInactive( int xPeripherals )
{
   taskENTER_CRITICAL();
   xDeepSleepForbiddenFlags &= ~xPeripherals;
   taskEXIT_CRITICAL();
}

void vUlpOnPeripheralsInactiveFromISR( int xPeripherals )
{
   UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
   xDeepSleepForbiddenFlags &= ~xPeripherals;
   taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


//      Functions vUlpPreSleepProcessing() and vUlpPostSleepProcessing() are called from a critical section,
// so we happily take a few shortcuts made safe by that usage model.
//
static uint32_t rccCfgrSave;
static uint32_t rccCrSave;
#ifdef SUPPORT_MSI_AT_48MHZ
static uint32_t rccIcscr1Save;
#endif
#ifdef SUPPORT_VREG_RANGES_1_THROUGH_3
static uint32_t pwrVosrSave;
#endif

void vUlpPreSleepProcessing()
{
   int useDeepSleep = pdFALSE;
   if (xDeepSleepForbiddenFlags == 0)
   {
      useDeepSleep = pdTRUE;
      #define PWR_CR1_LPMS_STOP2 PWR_CR1_LPMS_1
      MODIFY_REG(PWR->CR1, PWR_CR1_LPMS_Msk, PWR_CR1_LPMS_STOP2);
   }
   else if ((xDeepSleepForbiddenFlags & ~ulpPERIPHERALS_OK_IN_STOP1) == 0)
   {
      useDeepSleep = pdTRUE;
      #define PWR_CR1_LPMS_STOP1 PWR_CR1_LPMS_0
      MODIFY_REG(PWR->CR1, PWR_CR1_LPMS_Msk, PWR_CR1_LPMS_STOP1);
   }

   if (useDeepSleep)
   {
      rccCrSave = RCC->CR;
      rccCfgrSave = RCC->CFGR1;
      #ifdef SUPPORT_MSI_AT_48MHZ
      {
         rccIcscr1Save = RCC->ICSCR1;
      }
      #endif
      #ifdef SUPPORT_VREG_RANGES_1_THROUGH_3
      {
         pwrVosrSave = PWR->VOSR;
      }
      #endif

      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
   }
}

void vUlpPostSleepProcessing()
{
   if (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk)
   {
      //      We may have been in deep sleep.  If we were, the RCC cleared several enable bits in the CR, and
      // it changed the selected system clock in CFGR.  We need to restore them.

      //      Restore the clock-enable bits as described above.  Do it now in case we must wait below for the
      // PWR->VOSR register to indicate that the core-voltage booster is ready.  (It may require a clock that
      // is enabled by RCC->CR or a frequency set by RCC->ICSCR1.)
      //
      RCC->CR = rccCrSave;
      #ifdef SUPPORT_MSI_AT_48MHZ
      {
         RCC->ICSCR1 = rccIcscr1Save;
      }
      #endif

      #ifdef SUPPORT_VREG_RANGES_1_THROUGH_3
      {
         //      In the STM32U, we must first restore the previous regulator range because it may be required
         // to support the previous clock speeds.  In deep sleep, the PWR module sets range 4 automatically.
         // Wait for the regulator (and booster if necessary) to be ready before restoring the clocks.
         //
         uint32_t captureBefore = DWT->CYCCNT;

         do
         {
            PWR->VOSR = pwrVosrSave;
         } while (PWR->VOSR != pwrVosrSave);

         uint32_t duration = DWT->CYCCNT - captureBefore;

         //      If we're attaching the debugger after collecting min/max data without the debugger, then
         // don't let the debugger spoil the data by letting the application run a little bit (under the
         // debugger) during debug start.
         //
         if (xTaskGetTickCount() > pdMS_TO_TICKS(5000))
         {
            if (duration < minTime) minTime = duration;
            if (duration > maxTime) maxTime = duration;
         }
      }
      #endif

      //      Now restore the selected system clock.  If we've just restarted the PLL above and if we now
      // select it as the CPU clock, the CPU continues executing instructions on the wake-up clock (HSI or
      // MSIe) until the PLL is stable, and then the CPU starts using the PLL.
      //
      RCC->CFGR1 = rccCfgrSave;

      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

      //      This application bypasses the RTC shadow registers, so we don't need to clear the sync flag for
      // those registers.  They are always out of sync when coming out of deep sleep.
      //
      // RTC->ISR &= ~RTC_ISR_RSF;
   }
}
