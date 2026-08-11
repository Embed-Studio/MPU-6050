# MPU-6050 on STM32

Reference firmware for the EmbedStudio **MPU-6050 experiment series**. Each
article pins a release tag, so checking out a tag gives the firmware exactly as
that article describes it:

| Tag | Article | Firmware state |
| --- | --- | --- |
| `experiment_001` | [I²C Bring-Up](https://embed-studio.com/experiments/mpu6050/experiment-001-bringup) | init + raw sample reading, nothing else |
| `experiment_002` | [Six-Position Accelerometer Calibration](https://embed-studio.com/experiments/mpu6050/experiment-002-mpu6050-accelerometer-calibration) | adds the calibration model |

The firmware stays deliberately small: read the sensor, apply six constants, and
expose everything as live variables over SWD. No filtering, no sensor fusion.
Raw counts are never overwritten — the calibrated values live alongside them, so
a single capture carries both.

## Hardware

- **STM32F407G-DISC1** (STM32F407VGT6, 168 MHz)
- **GY-521 / MPU-6050** IMU module

| MPU-6050 | STM32F407 | Note |
| -------- | --------- | ---- |
| VCC | 3V3 | |
| GND | GND | |
| SCL | PB6 (I2C1_SCL) | 4.7 kΩ pull-up to 3V3 |
| SDA | PB9 (I2C1_SDA) | 4.7 kΩ pull-up to 3V3 |
| AD0 | GND | selects I²C address `0x68` |
| INT | PB8 (EXTI8) | data-ready interrupt, rising edge (active high) |

## What the firmware does

1. `MPU6050_Init()` waits for the sensor to answer (it needs ~100 ms from VDD,
   and an MCU reset does *not* power-cycle it), verifies `WHO_AM_I`, resets the
   device, then configures it:

   | Register | Addr | Value | Meaning |
   | --- | --- | --- | --- |
   | `PWR_MGMT_1` | `0x6B` | `0x03` | wake, PLL with Z-gyro reference |
   | `CONFIG` | `0x1A` | `0x03` | DLPF 44 Hz accel / 42 Hz gyro, 1 kHz base rate |
   | `SMPLRT_DIV` | `0x19` | `0x00` | output data rate 1 kHz |
   | `GYRO_CONFIG` | `0x1B` | `0x00` | ±250 °/s, 131 LSB/(°/s) nominal |
   | `ACCEL_CONFIG` | `0x1C` | `0x00` | ±2 g, 16384 LSB/g nominal |
   | `INT_ENABLE` | `0x38` | `0x01` | data-ready interrupt |

2. The sensor asserts DATA_RDY on PB8. The EXTI callback only sets a flag and
   timestamps the interval — no I²C in interrupt context.
3. The main loop polls the flag and does a blocking 15-byte burst read from
   `INT_STATUS`, which yields the data-ready flag, then accelerometer,
   temperature and gyroscope in one transfer. It also re-arms the flag whenever
   the INT pin reads high, as a missed-interrupt safety net.
4. `MPU6050_Apply_Calibration()` converts the raw counts to g, and
   `MPU6050_Gravity_Magnitude()` derives ‖a‖ from the result.
5. If the bus locks up, `MPU6050_Reset_I2C_Bus()` bit-bangs up to 9 SCL pulses to
   free a slave holding SDA low, force-resets the peripheral and re-initialises.
   This path is not decorative — the MPU-6050 does hang the bus in practice.
   It is the only code that needs to know the bus pins; they are `#ifndef`-guarded
   macros at the top of `mpu6050.h` (`MPU6050_SCL_Pin`, `MPU6050_SDA_GPIO_Port`, …)
   defaulting to I2C1 on PB6/PB9, so a different wiring is a define, not an edit.

### Why the burst starts at `INT_STATUS` and not at `ACCEL_XOUT_H`

Reading `ACCEL_XOUT_H` does not clear the MPU-6050's INT pin. `INT_PIN_CFG`
(`0x37`) is never written, so `INT_RD_CLEAR` stays 0 and **only a read of
`INT_STATUS` releases the interrupt**. The original firmware read neither, so the
INT pin stayed asserted, step 3's level poll re-armed the flag on every loop
iteration, and the read loop free-ran at whatever the I²C bus allowed instead of
at the sensor's data rate: **1998.7 reads/s (σ = 0.01)** measured over twelve
power cycles against a 999 Hz sensor — **2.00 reads per sensor update**, double
the necessary bus traffic, and a `sample_count` that counted bus transactions
rather than samples. Post-fix the same measurement reads **999.4 reads/s, exactly
one per update.**

`INT_STATUS` (`0x3A`) sits directly below `ACCEL_XOUT_H` (`0x3B`), so the fix is
free: read 15 bytes instead of 14, starting one register earlier. One extra byte
on the wire (~23 µs in Fast mode) replaces a whole second addressed transaction,
and the byte is not waste — `buffer[0] & MPU6050_INT_DATA_RDY` says whether the
14 that follow are a sample not seen before, which is exactly what
`mpu_data.sample_count` needs in order to count sensor updates.

The order matters. `INT_STATUS` is read **first**, not last: the data registers
are shadowed while the bus is busy, so a burst returns the sample set that was
current when it began. If the sensor updates mid-burst, the INT it raises is for
a sample this read did not return — clearing at the front leaves that interrupt
standing for the next pass, clearing at the end would wipe it and drop the
sample.

Two consequences elsewhere:

- **The EXTI edge is rising, not falling.** The INT pin is active high
  (`INT_PIN_CFG` bit 7 at its reset 0), so the assertion is the rising edge. With
  the interrupt now released on every read, a falling-edge trigger would fire on
  the firmware's *own* read and immediately re-arm the flag, restoring the double
  traffic this fix removes. It also means `sample_time_us` now measures
  update-to-update intervals; before the fix the pin never fell, so the EXTI
  never ran and the value was frozen.
- **`cnt_stale_reads`** counts bursts that came back with DATA_RDY clear. On a
  correctly paced loop it never moves, so it is the check that the pacing is
  right — watch it in a capture.

### Sample rate: 999 Hz measured, so the configured 1 kHz is right

`PWR_MGMT_1 = 0x03` selects the PLL with the Z-gyro as reference, so the sample
clock comes from the gyro oscillator rather than from a crystal — worth checking
rather than assuming. Measured over a 16-minute run on two independent clocks:

| Method | Clock | Result |
| --- | --- | --- |
| `sample_count` increments ÷ duration | host, over SWD | 999.39 /s |
| 1 ÷ mean `sample_time_us` (1001.0 µs) | MCU DWT, 168 MHz | 999.00 /s |

They agree to 0.04 %, and `sample_time_us` reads exactly 1001 µs on 961,128 of
961,214 samples with no longer interval anywhere, so no DATA_RDY edge was missed.
**The configured 1 kHz is correct to 0.1 % on this part.**

An earlier figure of ~1030 Hz (+3 %) is **wrong and withdrawn**. It came from
inferring the rate as `host_rate × (1 − identical_payload_fraction)` on the
pre-fix firmware, where the read loop free-ran at ~2× the sensor rate; a host
sampling at ~1050 /s stepped over two firmware writes at a time and almost never
saw the duplicate pair the method depends on, so the estimate simply tracked the
host rate. Both measurements above became possible only once the DATA_RDY fix
made `sample_count` and `sample_time_us` mean what they say.

## Calibration

One line per axis:

```c
acceleration[g] = (raw - bias) / scale
```

Six constants, all measured with the six-position method — point each axis up,
then down, average each position, then `bias = (raw_up + raw_down) / 2` and
`scale = (raw_up - raw_down) / 2`.

They are compiled in as `#ifndef`-guarded defaults in `mpu6050.h`, so you can
supply your own from the build (`-DMPU6050_ACCEL_SCALE_X_DEFAULT=...`) without
editing the driver:

| Axis | Bias (LSB) | Scale (LSB/g) | vs. nominal |
| --- | ---: | ---: | ---: |
| X | 197.35 | 16407.75 | +0.15 % |
| Y | −70.25 | 16401.25 | +0.11 % |
| Z | −37.50 | 16611.20 | +1.39 % |

**These belong to one specific module**, measured at a die temperature of
27.8–29.8 °C. They are a property of that part, not of the MPU-6050 — measure
your own.

`MPU6050_Calibration_Init()` loads them into an `MPU6050_Calibration_t` held in
RAM rather than into file-scope constants, so the six values can be retuned live
over SWD: changing a constant stays an edit rather than a rebuild.

Two caveats the measurements themselves showed:

- The **scale factors are stable** — they reproduced to better than 0.03 % across
  a power cycle, so compiling them in is safe.
- The **bias is not**. It was pinned to about ±3 LSB, but moved by up to 37 LSB
  (2.2 mg) across a single power cycle. A stored offset therefore describes the
  device as it was rather than as it is. If that error matters for your
  application, re-measure the bias at startup instead of trusting the value here.

`MPU6050_Gravity_Magnitude()` is the validation criterion: at rest it reads 1 g
whichever way the board is turned, so a wrong constant is visible live without
any reference hardware.

## SWD capture probes

Several file-scope globals exist purely to be **sampled live over SWD** —
`mpu_data`, `mpu_cal`, `accel_g`, `accel_magnitude_g`, `cnt_main_cycles`,
`cnt_read_failure`, `cnt_stale_reads`, `mpu_int_status`, `error_trap`,
`mpu6050_id`. They look unused
to the compiler and to a reader. Keep them non-`static`, and keep the Debug build
at `-Og`, or they get optimised away and the capture breaks.

`mpu_data.sample_count` is worth tracing alongside the measurements. It counts
**sensor updates** — only bursts that came back with DATA_RDY set — so a host can
tell a fresh sample from the same reading seen twice without comparing payloads.
It is written *after* every other field, so a host that sees it change is looking
at a fully written sample. It lives inside `MPU6050_t` rather than beside it so
the count cannot be separated from the data it counts, and it is `uint16_t` so it
costs only two bytes per sample on the SWD link (it wraps every ~65 s, which does
not matter for comparing adjacent samples).

Captures taken before the DATA_RDY fix have a `sample_count` that advances at the
I²C read rate, ~1.94× the sensor rate. In those, duplicated samples are found by
looking for byte-identical consecutive payloads across the six raw axes.

## Layout

The repository root holds the CubeMX-generated project (`MPU-6050.ioc`, `Core/`,
`Drivers/`). `STM32CubeIDE/` holds the Eclipse/CDT project that builds it; it has
no sources of its own but links them in:

| IDE virtual folder | Real path | Contents |
| --- | --- | --- |
| `Application/User/Core` | `STM32CubeIDE/Application/User/Core` | hand-written driver (`mpu6050.c/.h`) |
| `Application/libs/timer` | `timer/` | DWT cycle-counter timing module |
| (implicit) | `Core/`, `Drivers/` | CubeMX-generated |

Files under `Core/` are regenerated by CubeMX from the `.ioc` — only edit inside
the `/* USER CODE BEGIN X */ … /* USER CODE END X */` guards.

## Build

Open `STM32CubeIDE/` in STM32CubeIDE and build, or headlessly with the makefiles
the IDE generates:

```bash
export PATH="<stm32cube>/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH"
make -C STM32CubeIDE/Debug -j8 all     # -> STM32CubeIDE/Debug/MPU-6050.elf
```

Flash:

```bash
STM32_Programmer_CLI -c port=SWD -w STM32CubeIDE/Debug/MPU-6050.elf -rst
```

## Next in the series

Experiment #003 turns to the gyroscope — bias, drift, and how small errors
accumulate once they are integrated.
