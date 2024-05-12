// Ultra Low Power API implementation (ulp.c)

#include "stm32u5xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ulp.h"

// ULP Configuration Options
//
// #define SUPPORT_VREG_RANGES_1_THROUGH_3
// #define SUPPORT_MSI_AT_48MHZ

// Use of MSI as the clock source for the PLL
//
//      An ultra-low-power application that uses MSI as the clock source for the PLL must ensure that the MSI
// frequency is compatible with the flash wait-state and core-voltage regulator configurations as if MSI were
// providing the core clock directly.  In these applications, MSI does provide the core clock directly for a
// short time after waking from stop mode.  This is typically not an issue because the PLL is typically set
// for a frequency faster than its incoming MSI frequency.

// Warning about HSE and Stop modes
//
//      An ultra-low-power application that uses HSE should use it only when needed, and should avoid the use
// of stop modes while HSE is in use.  This advice applies even if HSE is used in bypass mode.  An easy way to
// avoid stop modes while HSE is in use is to register it as a peripheral not safe for stop modes.  See
// ulpPERIPHERAL_HSE in ulp.h for example.
//
//      This implementation of ULP assumes the application follows the advice above.  The clock configuration
// that doesn't use HSE is considered the ULP-friendly clock configuration.

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
   #define RCC_CFGR1_SW_MSIS   (0 * RCC_CFGR1_SW_0)
   #define RCC_CFGR1_SW_HSI16  (1 * RCC_CFGR1_SW_0)
   #define RCC_CFGR1_SW_HSE    (2 * RCC_CFGR1_SW_0)
   #define RCC_CFGR1_SW_PLL    (3 * RCC_CFGR1_SW_0)
   if ( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_HSI16 )
   {
      SET_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
   }
   else if ( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_MSIS )
   {
      CLEAR_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
   }
   else
   {
      //     An assertion failure here means the application has called vUlpInit() while HSE is providing the
      // core clock.  The application must call vUlpInit() while the ULP-friendly clock configuration is in
      // place.  See "Warning about HSE and Stop modes" above for more information.
      //
      configASSERT( (RCC->CFGR1 & RCC_CFGR1_SW_Msk) == RCC_CFGR1_SW_PLL );

      //      The application has selected the PLL (PLL1) as the system clock.  In this case, we optimize the
      // wake up timing and simplify the post-sleep code by configuring the MCU to wake from the source clock
      // for the PLL (MSIS or HSI16).
      //
      #define RCC_PLL1CFGR_PLL1SRC_MSIS   (1 * RCC_PLL1CFGR_PLL1SRC_0)
      #define RCC_PLL1CFGR_PLL1SRC_HSI16  (2 * RCC_PLL1CFGR_PLL1SRC_0)
      #define RCC_PLL1CFGR_PLL1SRC_HSE    (3 * RCC_PLL1CFGR_PLL1SRC_0)
      if ( (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC_Msk) == RCC_PLL1CFGR_PLL1SRC_HSI16 )
      {
         SET_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);
      }
      else
      {
         CLEAR_BIT(RCC->CFGR1, RCC_CFGR1_STOPWUCK);

         //      An assertion failure here means the application has called vUlpInit() while HSE is providing
         // the input clock to the PLL.  The application must call vUlpInit() while the ULP-friendly clock
         // configuration is in place.  See "Warning about HSE and Stop modes" above for more information.
         //
         configASSERT( (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC_Msk) == RCC_PLL1CFGR_PLL1SRC_MSIS );
      }
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
      // it changed the selected system clock in CFGR.  Restore them now.  If we're restarting the PLL as
      // the CPU clock here, the CPU will not wait for it.  Instead, the CPU continues executing from the
      // wake-up clock (MSIS or HSI16) until the PLL is stable and then the CPU starts using the PLL.

      #ifdef SUPPORT_VREG_RANGES_1_THROUGH_3
      {
         //      In the STM32U5, we must first restore the previous regulator range because it may be required
         // to support the previous clock speeds.  In deep sleep, the PWR module sets range 4 automatically.
         // Wait for the regulator (and booster if necessary) to be ready before restoring the clocks.  Note
         // that the booster clock is also the wake-up clock by design, so that clock is running already.
         //
         do
         {
            PWR->VOSR = pwrVosrSave;
         } while (PWR->VOSR != pwrVosrSave);
      }
      #endif

      #ifdef SUPPORT_MSI_AT_48MHZ
      {
         RCC->ICSCR1 = rccIcscr1Save;  // Safe.  Either RCC_CR_MSISRDY is set here, or RCC_CR_MSISON is clear.
      }
      #endif

      RCC->CR = rccCrSave;
      RCC->CFGR1 = rccCfgrSave;

      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

      //      This application bypasses the RTC shadow registers, so we don't need to clear the sync flag for
      // those registers.  They are always out of sync when coming out of deep sleep.
      //
      // RTC->ISR &= ~RTC_ISR_RSF;
   }
}
