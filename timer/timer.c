/**
  ******************************************************************************
  * @file    timer.c
  * @author  oleg
  * @date    2026-Jul-27
  * @brief   Reusable high-resolution timing module based on the Cortex-M DWT
  *          cycle counter - initialisation and cached conversion factors.
  ******************************************************************************
  * @details
  * Everything on the hot path (@ref timer_now, @ref timer_elapsed, the
  * conversions and the interval helper) is inline in timer.h. This translation
  * unit only holds one-time setup and the cached constants they use.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "timer.h"

/* Private define ------------------------------------------------------------*/

/**
  * @brief Cortex-M7 DWT software lock: address of the Lock Access Register.
  *
  * ARMv7-M cores with the software lock implemented (Cortex-M7) ignore writes
  * to the DWT registers until the unlock key is written to DWT_LAR. The
  * register is not part of the CMSIS @c DWT_Type on all core headers, so it is
  * addressed directly. Cortex-M3/M4 do not implement the lock and ignore the
  * write to this reserved location.
  */
#define TIMER_DWT_LAR       (*(volatile uint32_t *)(DWT_BASE + 0xFB0UL))

/** @brief Unlock key for #TIMER_DWT_LAR. */
#define TIMER_DWT_LAR_KEY   0xC5ACCE55UL

/* Exported variables --------------------------------------------------------*/

/**
  * @brief Core clock frequency cached by @ref timer_init(), in Hz.
  *
  * Zero until initialisation, which makes the integer conversions return 0
  * instead of dividing by zero when the module is used uninitialised.
  */
uint32_t timer_core_clock_hz = 0u;

/**
  * @brief Precomputed cycle-to-time scale factors used by the float conversions.
  *
  * Zero until initialisation, so uninitialised use yields 0.0f rather than a
  * plausible but wrong value.
  */
float timer_scale_ns_f = 0.0f;
float timer_scale_us_f = 0.0f;
float timer_scale_ms_f = 0.0f;
float timer_scale_s_f  = 0.0f;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialise the DWT cycle counter and the conversion factors.
  * @details Sequence:
  *          1. enable the trace subsystem (@c DEMCR.TRCENA), required before
  *             any DWT register is writable;
  *          2. unlock the DWT (Cortex-M7 software lock, harmless elsewhere);
  *          3. clear @c CYCCNT and enable it;
  *          4. cache @c SystemCoreClock and derive the scale factors.
  *
  *          If the core reports @c DWT_CTRL.NOCYCCNT the cycle counter is not
  *          implemented: the counter cannot be enabled and all timestamps stay
  *          at zero. The conversion factors are still set up so that the API
  *          remains usable (every measured interval reads as zero).
  * @see timer.h for the full contract.
  */
void timer_init(void)
{
    float cycles_per_second;

    /* 1. Enable trace and debug blocks; without TRCENA the DWT ignores writes. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 2. Release the DWT software lock (Cortex-M7); no-op on M3/M4. */
    TIMER_DWT_LAR = TIMER_DWT_LAR_KEY;

    /* 3. Restart the cycle counter. */
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 4. Cache the clock and precompute the runtime-division-free factors. */
    timer_core_clock_hz = SystemCoreClock;

    if (timer_core_clock_hz != 0u)
    {
        cycles_per_second = (float)timer_core_clock_hz;

        timer_scale_s_f  = 1.0f / cycles_per_second;
        timer_scale_ms_f = 1.0e3f * timer_scale_s_f;
        timer_scale_us_f = 1.0e6f * timer_scale_s_f;
        timer_scale_ns_f = 1.0e9f * timer_scale_s_f;
    }
}
