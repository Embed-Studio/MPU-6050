/**
  ******************************************************************************
  * @file    timer.h
  * @author  oleg
  * @date    2026-Jul-27
  * @brief   Reusable high-resolution timing module based on the Cortex-M DWT
  *          cycle counter.
  ******************************************************************************
  * @details
  *
  * The module provides cycle-accurate timestamps with near-zero overhead by
  * reading the Data Watchpoint and Trace (DWT) unit cycle counter
  * (@c DWT->CYCCNT). It does not use HAL timers, SysTick, interrupts, an RTOS
  * or any peripheral, and performs no background processing whatsoever.
  *
  * Typical uses: code profiling, execution-time budgeting, sensor sampling
  * intervals, numerical integration time steps and runtime diagnostics.
  *
  * @section timer_repr Time representation
  *
  * The primary representation is CPU cycles ("ticks"), see #timer_ticks_t.
  * Conversion to nanoseconds/microseconds/milliseconds/seconds is performed
  * only when a human-readable or physical value is actually needed.
  *
  * @section timer_req Requirements
  *
  * - A Cortex-M core that implements the DWT unit with a cycle counter:
  *   Cortex-M3, Cortex-M4, Cortex-M7 (also Cortex-M33/M55 with DWT present).
  *   Cortex-M0/M0+ do **not** implement DWT and are not supported.
  * - CMSIS device header providing @c DWT, @c CoreDebug and @c SystemCoreClock.
  * - @ref timer_init() must be called once after the system clock is
  *   configured (i.e. after @c SystemClock_Config() / @c SystemCoreClockUpdate()).
  *
  * @note On some devices the DWT is held in reset or powered down until a
  *       debugger attaches. @ref timer_init() enables @c TRCENA itself and
  *       unlocks the DWT on Cortex-M7, so a debugger is normally not required.
  *       If @c DWT->CTRL reports @c NOCYCCNT the cycle counter is unavailable
  *       and all timestamps read as zero.
  *
  * @section timer_wrap Wraparound
  *
  * @c CYCCNT is a free-running 32-bit up-counter that wraps after 2^32 cycles:
  *
  * | Core clock | Wraparound period (2^32 cycles) |
  * |------------|---------------------------------|
  * |  16 MHz    | ~268.44 s                       |
  * |  72 MHz    | ~59.65 s                        |
  * |  84 MHz    | ~51.13 s                        |
  * | 168 MHz    | ~25.56 s  (STM32F4 @ 168 MHz)   |
  * | 216 MHz    | ~19.88 s                        |
  * | 480 MHz    | ~8.95 s                         |
  *
  * Wraparound does **not** affect interval measurements: @ref timer_elapsed()
  * subtracts two @c uint32_t values, and unsigned arithmetic in C is defined
  * modulo 2^32, so the difference is exact as long as the measured interval is
  * shorter than one wraparound period. Only intervals *longer* than the period
  * above are ambiguous (they alias modulo 2^32).
  *
  * @section timer_usage Usage
  *
  * Basic interval measurement:
  * @code{.c}
  * timer_init();                              // once, after clock config
  *
  * timer_ticks_t t0 = timer_now();
  * MPU6050_Read_All(&hi2c1, &mpu_data);
  * timer_ticks_t dt = timer_elapsed(t0);
  *
  * uint64_t us  = timer_us(dt);               // exact, integer
  * float    ms  = timer_ms_f(dt);             // fast, float
  * @endcode
  *
  * Profiling with the stopwatch macros:
  * @code{.c}
  * TIMER_START(read_sensor);
  * MPU6050_Read_All(&hi2c1, &mpu_data);
  * uint64_t us = timer_us(TIMER_ELAPSED(read_sensor));
  * @endcode
  *
  * Sample-to-sample interval in an acquisition loop:
  * @code{.c}
  * static timer_interval_t sample_timer;
  * timer_interval_init(&sample_timer);        // once, before the loop
  *
  * while (1)
  * {
  *     if (mpu_data_ready)
  *     {
  *         sample.cycles = timer_interval_update(&sample_timer);
  *         sample.dt     = timer_s_f(sample.cycles);
  *         angle_z      += gyro_z * sample.dt;
  *     }
  * }
  * @endcode
  ******************************************************************************
  */
#ifndef TIMER_TIMER_H_
#define TIMER_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/

#include <stdint.h>

/**
  * @brief CMSIS device header selection.
  *
  * The module needs the device header that pulls in the CMSIS core definitions
  * (@c DWT, @c CoreDebug, @c SystemCoreClock). Define @c TIMER_CMSIS_HEADER on
  * the compiler command line to select it explicitly, e.g.
  * @c -DTIMER_CMSIS_HEADER="\"stm32f4xx.h\"" . Otherwise the common STM32
  * family headers are probed automatically.
  */
#if defined(TIMER_CMSIS_HEADER)
#  include TIMER_CMSIS_HEADER
#elif defined(__has_include)
#  if   __has_include("stm32f4xx.h")
#    include "stm32f4xx.h"
#  elif __has_include("stm32f7xx.h")
#    include "stm32f7xx.h"
#  elif __has_include("stm32f3xx.h")
#    include "stm32f3xx.h"
#  elif __has_include("stm32f2xx.h")
#    include "stm32f2xx.h"
#  elif __has_include("stm32f1xx.h")
#    include "stm32f1xx.h"
#  elif __has_include("stm32h7xx.h")
#    include "stm32h7xx.h"
#  elif __has_include("stm32g4xx.h")
#    include "stm32g4xx.h"
#  elif __has_include("stm32l4xx.h")
#    include "stm32l4xx.h"
#  elif __has_include("stm32l5xx.h")
#    include "stm32l5xx.h"
#  elif __has_include("stm32u5xx.h")
#    include "stm32u5xx.h"
#  elif __has_include("stm32wbxx.h")
#    include "stm32wbxx.h"
#  else
#    error "timer: cannot locate CMSIS device header, define TIMER_CMSIS_HEADER"
#  endif
#else
#  error "timer: compiler lacks __has_include, define TIMER_CMSIS_HEADER"
#endif

#if !defined(DWT_BASE)
#  error "timer: target core does not implement the DWT unit (Cortex-M0/M0+?)"
#endif

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Timestamp / interval expressed in CPU core clock cycles.
  *
  * This is the native DWT counter width. All timestamps and intervals are
  * carried in this type; differences are computed with unsigned (modulo 2^32)
  * arithmetic and are therefore wraparound-safe, see @ref timer_wrap.
  */
typedef uint32_t timer_ticks_t;

/**
  * @brief Periodic interval helper state.
  *
  * Holds the timestamp of the previous @ref timer_interval_update() call.
  * Contains no dynamic memory and may live in static storage, on the stack or
  * inside an application structure.
  */
typedef struct
{
    timer_ticks_t last; /**< Timestamp of the previous update, in cycles. */
} timer_interval_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief Cached core clock frequency in Hz, sampled by @ref timer_init().
  * @note Read-only for application code. Zero until @ref timer_init() runs.
  */
extern uint32_t timer_core_clock_hz;

/**
  * @brief Precomputed scale factors, one multiplication per float conversion.
  * @note Read-only for application code. Zero until @ref timer_init() runs.
  */
extern float timer_scale_ns_f; /**< Nanoseconds  per cycle. */
extern float timer_scale_us_f; /**< Microseconds per cycle. */
extern float timer_scale_ms_f; /**< Milliseconds per cycle. */
extern float timer_scale_s_f;  /**< Seconds      per cycle. */

/* Exported macro ------------------------------------------------------------*/

/**
  * @brief Declare and start a named stopwatch.
  *
  * Expands to a local @c timer_ticks_t variable holding the current timestamp,
  * so the name only has to be unique within the enclosing scope.
  *
  * @param name Identifier of the stopwatch (a valid C identifier fragment).
  *
  * @code{.c}
  * TIMER_START(read_sensor);
  * MPU6050_Read_All(&hi2c1, &mpu_data);
  * uint64_t us = timer_us(TIMER_ELAPSED(read_sensor));
  * @endcode
  */
#define TIMER_START(name)   timer_ticks_t timer_sw_##name = timer_now()

/**
  * @brief Cycles elapsed since the matching @ref TIMER_START.
  * @param name Identifier used in @ref TIMER_START.
  * @return Elapsed cycles as #timer_ticks_t.
  * @note May be evaluated repeatedly; the stopwatch keeps running.
  */
#define TIMER_ELAPSED(name) timer_elapsed(timer_sw_##name)

/**
  * @brief Restart a running stopwatch at the current timestamp.
  * @param name Identifier used in @ref TIMER_START.
  */
#define TIMER_RESTART(name) (timer_sw_##name = timer_now())

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialise the DWT cycle counter and the conversion factors.
  *
  * Enables trace (@c DEMCR.TRCENA), unlocks the DWT where required, clears
  * @c CYCCNT, enables @c CYCCNT, caches @c SystemCoreClock and precomputes the
  * floating-point scale factors.
  *
  * Call once during system startup, after the core clock is configured and
  * @c SystemCoreClock holds the final frequency. Calling it again is harmless
  * but resets the counter to zero, which invalidates outstanding timestamps.
  *
  * @note If the core clock is changed at runtime, call this function again.
  * @warning Not thread-safe / not reentrant; call it before starting any
  *          concurrent user of the module.
  */
void timer_init(void);

/**
  * @brief Read the current cycle-counter value.
  * @return Current @c DWT->CYCCNT value in cycles.
  *
  * Compiles to a single load from the DWT (a few cycles). The value is free
  * running and wraps every 2^32 cycles, see @ref timer_wrap.
  */
__STATIC_INLINE timer_ticks_t timer_now(void)
{
    return (timer_ticks_t)DWT->CYCCNT;
}

/**
  * @brief Cycles elapsed since a previously captured timestamp.
  * @param start Timestamp obtained from @ref timer_now().
  * @return Elapsed cycles.
  *
  * Uses unsigned modulo-2^32 subtraction and is therefore correct across a
  * counter wraparound, provided the measured interval is shorter than one
  * wraparound period (see @ref timer_wrap).
  *
  * @code{.c}
  * timer_ticks_t t0 = timer_now();
  * // code under test
  * timer_ticks_t dt = timer_elapsed(t0);
  * @endcode
  */
__STATIC_INLINE timer_ticks_t timer_elapsed(timer_ticks_t start)
{
    return (timer_ticks_t)(timer_now() - start);
}

/**
  * @brief Convert cycles to nanoseconds (integer, rounded to nearest).
  * @param ticks Interval in cycles.
  * @return Interval in nanoseconds, or 0 if @ref timer_init() has not run.
  *
  * Evaluated in 64-bit integer arithmetic with no floating point; the result is
  * exact to within half a nanosecond. Involves one 64-bit division, so prefer
  * @ref timer_ns_f() or raw cycles inside tight loops.
  */
__STATIC_INLINE uint64_t timer_ns(timer_ticks_t ticks)
{
    const uint32_t f = timer_core_clock_hz;

    if (f == 0u)
    {
        return 0u;
    }

    return (((uint64_t)ticks * 1000000000ULL) + (uint64_t)(f / 2u)) / (uint64_t)f;
}

/**
  * @brief Convert cycles to microseconds (integer, rounded to nearest).
  * @param ticks Interval in cycles.
  * @return Interval in microseconds, or 0 if @ref timer_init() has not run.
  * @see timer_ns()
  */
__STATIC_INLINE uint64_t timer_us(timer_ticks_t ticks)
{
    const uint32_t f = timer_core_clock_hz;

    if (f == 0u)
    {
        return 0u;
    }

    return (((uint64_t)ticks * 1000000ULL) + (uint64_t)(f / 2u)) / (uint64_t)f;
}

/**
  * @brief Convert cycles to milliseconds (integer, rounded to nearest).
  * @param ticks Interval in cycles.
  * @return Interval in milliseconds, or 0 if @ref timer_init() has not run.
  * @see timer_ns()
  */
__STATIC_INLINE uint64_t timer_ms(timer_ticks_t ticks)
{
    const uint32_t f = timer_core_clock_hz;

    if (f == 0u)
    {
        return 0u;
    }

    return (((uint64_t)ticks * 1000ULL) + (uint64_t)(f / 2u)) / (uint64_t)f;
}

/**
  * @brief Convert cycles to nanoseconds (single precision).
  * @param ticks Interval in cycles.
  * @return Interval in nanoseconds, or 0.0f if @ref timer_init() has not run.
  *
  * One multiplication by a precomputed factor; no runtime division. Single
  * precision carries 24 significant bits, so intervals above ~16.7 million
  * cycles are rounded (relative error stays below 2^-24, ~6e-8).
  */
__STATIC_INLINE float timer_ns_f(timer_ticks_t ticks)
{
    return (float)ticks * timer_scale_ns_f;
}

/**
  * @brief Convert cycles to microseconds (single precision).
  * @param ticks Interval in cycles.
  * @return Interval in microseconds, or 0.0f if @ref timer_init() has not run.
  * @see timer_ns_f()
  */
__STATIC_INLINE float timer_us_f(timer_ticks_t ticks)
{
    return (float)ticks * timer_scale_us_f;
}

/**
  * @brief Convert cycles to milliseconds (single precision).
  * @param ticks Interval in cycles.
  * @return Interval in milliseconds, or 0.0f if @ref timer_init() has not run.
  * @see timer_ns_f()
  */
__STATIC_INLINE float timer_ms_f(timer_ticks_t ticks)
{
    return (float)ticks * timer_scale_ms_f;
}

/**
  * @brief Convert cycles to seconds (single precision).
  * @param ticks Interval in cycles.
  * @return Interval in seconds, or 0.0f if @ref timer_init() has not run.
  *
  * Intended as the time step of numerical integration:
  * @code{.c}
  * float dt = timer_s_f(cycles);
  * angle += gyro_z * dt;
  * @endcode
  */
__STATIC_INLINE float timer_s_f(timer_ticks_t ticks)
{
    return (float)ticks * timer_scale_s_f;
}

/**
  * @brief Arm an interval helper at the current timestamp.
  * @param timer Interval helper to initialise. Must not be @c NULL.
  *
  * The first @ref timer_interval_update() then returns the time elapsed since
  * this call.
  */
__STATIC_INLINE void timer_interval_init(timer_interval_t *timer)
{
    timer->last = timer_now();
}

/**
  * @brief Sample the interval since the previous update and rearm.
  * @param timer Interval helper previously passed to @ref timer_interval_init().
  *              Must not be @c NULL.
  * @return Cycles elapsed since the previous update (or since init).
  *
  * Captures a single timestamp, so the returned intervals tile the timeline
  * without drift or gaps: consecutive results sum to the total elapsed time.
  *
  * @code{.c}
  * sample.cycles = timer_interval_update(&sample_timer);
  * sample.dt     = timer_s_f(sample.cycles);
  * @endcode
  */
__STATIC_INLINE timer_ticks_t timer_interval_update(timer_interval_t *timer)
{
    const timer_ticks_t now     = timer_now();
    const timer_ticks_t elapsed = (timer_ticks_t)(now - timer->last);

    timer->last = now;

    return elapsed;
}

#ifdef __cplusplus
}
#endif

#endif /* TIMER_TIMER_H_ */
