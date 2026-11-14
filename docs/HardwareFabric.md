# RA4M1 hardware fabric (firmware v4)

Scope: how the 500 Hz control path is paced, sampled and actuated by RA4M1 peripherals instead of a `micros()` polling loop. The control law (`firmware/src/admittance.h`), all simulation code and all datasets are unchanged. **Nothing here has run on silicon.** Every statement is either a citation into the Arduino `renesas_uno` 1.6.0 core (paths relative to `packages/arduino/hardware/renesas_uno/1.6.0/`), a register macro from `R7FA4M1AB.h`, or output from a command listed in section 8.

"FPGA-like" here means a fixed event chain wired in hardware: sampling instants come from a timer event routed by the ELC, not from software. Software still runs the control law in one ISR.

---

## 1. Audit of the starting point (v2)

| Item | v2 behaviour | Source |
|---|---|---|
| Pacing | `loop()` compares `micros()` against `next_tick_us`. Jitter is set by everything else running in `loop()` | v2 `src.ino` |
| Grip ADC | `analogRead()` ×4 in software. The core re-opens the ADC through FSP on every call | `cores/arduino/analog.cpp` |
| Encoder | GPT1 hardware 4× phase counting, read with `GTCNT` at an arbitrary time inside the tick | `ra4m1_registers.h` |
| PWM | `PwmOut` on D6 (GPT3 channel A, saw-wave) | `pwm.h` |
| Contact | `digitalRead(7)` once per tick, so contacts shorter than 2 ms can be missed | v2 `src.ino` |
| Fail-safe | Software only: STBY low unless armed | v2 `src.ino` |

## 2. Facts that constrain the design (and corrections to the brief)

| # | Fact | Evidence |
|---|---|---|
| F1 | Under `-DNO_USB`, `Serial` = `_UART1_` (SCI9, P109/P110, wired to the ESP32-S3 USB bridge), `Serial1` = `_UART2_` (D0/D1), `Serial2` = `_UART3_` | `boards.txt:122`, `cores/arduino/Arduino.h:113-115` |
| F2 | WiFiS3 talks AT commands to the ESP32-S3 over `Serial2` at 115200 baud. Calls block for up to `MODEM_TIMEOUT` = 10000 ms | `libraries/WiFiS3/src/Modem.cpp:424`, `Modem.h:12`, `Modem.h:54` |
| F3 | **Correction:** the RA4M1↔ESP32 data link is not "Serial1 through a TXB0108". `Serial1` is the D0/D1 header. The level-shifter part cannot be verified from files on this machine | F1 |
| F4 | `initVariant()` reserves the GPT channels of D3, D5, D6, D9, D10, D11 for PWM (GPT1, GPT0, GPT3, GPT7, GPT2, GPT6). GPT4 and GPT5 are free | `variants/UNOWIFIR4/variant.cpp:117-133`, `pinmux.inc` |
| F5 | `FspTimer::get_available_timer` hands out the first free GPT, then AGT. The LED matrix asks for a 10 kHz timer at `matrix.begin()` | `cores/arduino/FspTimer.cpp:133`, `libraries/Arduino_LED_Matrix/src/Arduino_LED_Matrix.h:172` |
| F6 | **Correction:** AGT0 drives `millis()`/`micros()` (1 ms, PCLKB/8, IRQ priority 8) but is not marked used, so it must not be reused as a pacer | `cores/arduino/time.cpp:28-43`, `FspTimer.cpp:16` (line commented out) |
| F7 | **Correction:** P112 (D7) has no `PIN_INTERRUPT` entry, so `attachInterrupt(7)` is impossible. P112 is GTIOC3B, so GPT3 can capture its edges in hardware | `pinmux.inc:149-153` |
| F8 | `IRQManager::addGenericInterrupt` writes any ELC event into `IELSR[n]` and installs the vector | `cores/arduino/IRQManager.cpp:42-63` |
| F9 | **Correction:** the IWDT cannot be enabled from software. Its start is set by OFS0, and the core's OFS0 = 0xFFFFFFFF (IWDT off, WDT in register-start mode). The WDT is used instead | `bsp_rom_registers.o` `.rom_registers` (objdump); `R_IWDT` has only `IWDTRR`/`IWDTSR` |
| F10 | **Correction:** the DAC cannot be routed internally to the op-amp or ADC (`BSP_FEATURE_DAC_HAS_INTERNAL_OUTPUT` = 0). The DAC pin P014 is also AN009, so a loopback works on the same pin with no wiring | `bsp_feature.h`; `pinmux.inc:238-240`; `pins_arduino.h:24` (`DAC` = A0) |
| F11 | Clocks: HOCO 48 MHz; PCLKD (GPT) = 48 MHz, PCLKC (ADC) = 48 MHz, PCLKB (WDT) = 24 MHz | `variants/UNOWIFIR4/includes/ra_gen/bsp_clock_cfg.h` |
| F12 | ELC: GPT0 overflow = event 93, ADC0 scan end = 41. ELSR slots: `ELC_PERIPHERAL_GPT_A` = 0, `ELC_PERIPHERAL_ADC0` = 8 | `bsp_elc.h:91,142`, `r_elc_api.h` |
| F13 | ADC ELC start: FSP `adc_elc_trigger_lut` gives `ADSTRGR` = 0x090A/0x0B3F, so TRSA = 0x09 is the ELC trigger. Group B is off at 0x3F | disassembly of `r_adc.o` from `libfsp.a` |
| F14 | No ESP32 core is installed on this machine, so `esp32_bridge.ino` cannot be compiled here | `arduino-cli core list` |

## 3. Resource allocation

| Resource | Use | Key settings |
|---|---|---|
| GPT0 (32-bit) | 500 Hz pacer, no pin | saw-wave, `GTPR` = 95999 (`HW_PACER_PERIOD - 1`) |
| GPT1 (32-bit) | Encoder, 4× phase counting on P105/P104 | `GTUPSR` 0x6900, `GTDNSR` 0x9600, noise filter PCLKD/64, **`GTICASR` bit 16** (ELC_GPT_A) latches `GTCNT` into `GTCCRA` |
| GPT3 (16-bit) | Motor PWM on GTIOC3A (P111, D6) and contact capture on GTIOC3B (P112, D7) | `GTCR.MD` = 4 (triangle, transfer at trough), `GTPR` = 1200 (20 kHz), `GTBER.CCRA` = single buffer, `GTIOR` = GTIOA 0x03 + OAE + OADF = LOW + NFBEN/NFCSB; `GTICBSR` BSCBFAL\|BSCBFAH (falling edges) |
| GPT4 | LED matrix 10 kHz (assigned by `FspTimer`, expected `GTPR` = 4799; printed as INFO at boot) | core library |
| AGT0 | `millis()`, untouched | core |
| ELC | ELSR0 = 93 (→ GPT1 capture), ELSR8 = 93 (→ ADC start), `ELCR.ELCON` = 1 | |
| ADC14 | AN002 only, single scan, ELC trigger, 4× hardware average, `ADIE` = 1 | `ADCER.ADPRC` = 3, `ADADC` = 0x83, `ADSTRGR` = 0x093F, `ADSSTR02` = 48 states (1.0 µs) |
| NVIC slot (`IELSR[n]`) | ADC0_SCAN_END → `controlIsr()`, priority 1 | `addGenericInterrupt` |
| POEG group A | Hardware output disable of GTIOC3A (forced LOW) | `GTINTAD.GRP` = 0, trip with `POEGG.SSF` (bit 3) |
| WDT | Reset on underflow if the control ISR stops | `WDTCR` = 0x3341: TOPS 4096 cycles, CKS PCLKB/64, window 0–100 %, → 10.92 ms; `WDTRCR.RSTIRQS` = 1 |
| PORT1/PORT3 `PCNTR3` | AIN1 (P106), AIN2 (P107), STBY (P304) | atomic set/reset, no read-modify-write |
| DAC12, OPAMP0, TSN, internal reference | Boot analog self-test only (section 6) | |

Module-stop bits released: MSTPCRD bit 5 (GPT0–1), bit 6 (GPT2–7), bit 14 (POEG), bit 16 (ADC), bit 22 (TSN), bit 31 (OPAMP); MSTPCRC bit 14 (ELC). The boot self-test reads them back.

No DTC is used. The two samples the ISR needs (encoder count and ADC result) are already latched in peripheral registers at the event instant, so moving them to RAM would not change their timing.

## 4. Event chain

```
           PCLKD 48 MHz
                |
   +------------v-------------+   same GTSTR write    +-----------------------------+
   | GPT0 saw 0..95999 (2 ms) |<--------------------->| GPT3 triangle 0..1200..0    |
   +------------+-------------+                       | 20 kHz, 40 periods per tick |
                | overflow = ELC event 93             | GTIOC3A (P111) -> PWMA      |
        +-------+---------+                           | GTIOC3B (P112) <- contact   |
        v                 v                           +------^----------+-----------+
   ELSR0 (GPT_A)     ELSR8 (AD00)                            | GTCCRC   | TCFB flag
        |                 |                                  | (buffer) |
        v                 v                                  |          |
  GPT1 GTICASR b16   ADC14 AN002 x4 avg                      |          |
  GTCNT -> GTCCRA    (sample at PWM trough)                  |          |
        |                 | ADC0_SCAN_END (event 41)         |          |
        |                 v                                  |          |
        |         IELSR[n] -> NVIC prio 1                    |          |
        |                 v                                  |          |
        +-------> controlIsr():  read GTCCRA, ADDR[2], TCFB -+----------+
                  kick WDT, HapticEngine::step() (admittance.h, unchanged)
                  -> GTCCRC (applied at the next trough), AIN1/AIN2/STBY via PCNTR3
                  -> seqlock snapshot for loop(); overrun -> POEGG.SSF
```

Because 96000 = 40 × 2400, every GPT0 overflow coincides with a GPT3 trough (`static_assert` in `config.h`). With GTIOA = 0x03 (toggle at compare match, initial LOW), the output is LOW while `GTCNT` < `GTCCRA`, so the trough is the middle of the PWM off-time. The grip sample is therefore taken while the H-bridge is not switching. `GTCCRA` is kept in [2, 1198] or set to 1201 (no match, output stays LOW). `GTCCRA` = 0 or = `GTPR` would give a single match per period and invert the toggle phase. The maximum duty is 99.8 %.

## 5. Fail-safe chain

| Layer | Trigger | Action | Release |
|---|---|---|---|
| Software arming | self-test, IRQ install or polarity test not passed | STBY LOW, AIN1 = AIN2 = LOW (coast) | pass the tests |
| Engine fault (unchanged) | `HapticEngine` fault latch (e.g. encoder jump) | coast, as in v2 | `R` / mode command |
| POEG (hardware latch) | the control ISR ends after the next pacer overflow (lost tick) | `POEGG.SSF` → GTIOC3A forced LOW. With AIN1/AIN2 still set, the TB6612 short-brakes (passive) | reset only |
| WDT | no refresh for 10.92 ms (≈5 ticks): ISR stopped, NVIC disabled, hard fault loop | MCU reset. All GPIO return to Hi-Z inputs | automatic |

After a reset, PWMA/STBY float until `setup()` runs. **External 10 kΩ pull-downs on STBY and PWMA are recommended.** The TB6612FNG's internal pull-downs could not be verified from local files. A brown-out or a WDT reset therefore leaves the motor unpowered (coast).

## 6. Boot self-test (before the pipeline starts, software-triggered ADC)

| Test | Method | Pass window |
|---|---|---|
| AVCC0 | Convert the internal reference (`ADEXICR.OCSA`, `ADOCDR`, 5.3 µs sampling) and compute AVCC = 1.43 V × 16383 / code (`AR_INTERNAL_VOLTAGE` = 1.43 V) | 4.5–5.5 V |
| DAC → ADC | `analogWrite(DAC)` 12-bit codes 1024/2048/3072, read back on AN009 (same pin P014). Expected ADC = 4 × code. **A0 must be unconnected** | ±2 % FS |
| OPAMP0 follower | AN002 (output) vs AN000 (input) | ±2 % FS |
| Temperature sensor | raw `ADTSDR` and the factory calibration word `TSCDRH:L` | informational |
| Registers | MSTP bits, PFS of P104/P105/P111/P112/P002, GPT0/1/3 configuration, ELSR0/8, OPAMP | exact |
| IRQ | `addGenericInterrupt` returned true | |

The register self-test (`S` command) only reads registers, so it is safe while the pipeline runs. The analog tests run only at boot, because a software-triggered conversion would collide with the ELC-triggered scan.

## 7. Telemetry and the dual-MCU path

- `loop()` formats rows from a seqlock snapshot and may block. Because control runs in the ISR, blocking no longer affects control timing. The v2 byte-drip queue is removed. `#STAT` now reports `max_isr_us`, `max_latency_us` (pacer overflow to ISR entry, which includes the ADC conversion) and `poeg`. Drops are rows skipped because `loop()` was late.
- `SG_TELEMETRY_SINK` (`config.h`): 1 = USB `Serial` (default), 2 = WiFi UDP through WiFiS3 (stock ESP32-S3 firmware, F2), 3 = both. Credentials come from a git-ignored `firmware/src/secrets.h` (`SECRET_SSID`, `SECRET_PASS`, optional `SECRET_UDP_HOST`). The firmware sends 5 rows per datagram to port 5005. The WiFiS3 calls block `loop()` only.
- `firmware/esp32_bridge/esp32_bridge.ino` is an **optional** replacement ESP32-S3 firmware. It forwards the CSV lines arriving from RA4M1 `Serial` as UDP. It cannot be compiled here (F14). Its link GPIOs are set to −1 with a `static_assert`, so it will not build until the pins are taken from the official schematic. Flashing it removes USB upload to the RA4M1 and WiFiS3 until the stock ESP32 firmware is restored.

## 8. Timing budget and verification

Static bound, computed from the linked image (`python telemetry/scripts/arm_cycle_budget.py --elf <build>/src.ino.elf`, Cortex-M4 TRM costs, ×2 for one flash wait state, all branches charged):

| Chain | Rate, priority | Bound | CPU share (bound) |
|---|---|---|---|
| `controlIsr()` including `HapticEngine::step` (inlined), plus 58 cycles of exception entry/exit and lazy FP stacking | 500 Hz, 1 | 3498 cycles = **72.9 µs** (≤ 250 µs: PASS), no loops, no double-precision calls | 3.64 % |
| `gpt_counter_overflow_isr` → `turnOnLedISR` (LED matrix) | 10 kHz, 12 | 1366 cycles = 28.5 µs | 28.46 % |
| `agt_int_isr` → `timer_micros_callback` (`millis`) | 1 kHz, 8 | 356 cycles = 7.4 µs | 0.74 % |

Lower-priority ISRs cannot delay the control ISR (priority 1 preempts them). Not bounded statically: the ADC conversion time (it is in `max_latency_us`), and the time any core or FSP code runs with interrupts globally disabled. Both are reported at runtime by `#STAT`. The two indirect calls the analyser lists are the FSP timer callback (analysed as its own root above) and the LED matrix `_callBack`, which is null because this firmware does not set it. The HOCO has ±1 % frequency tolerance, so the 500 Hz rate and the 20 kHz PWM carry the same tolerance.

From the pacer event to an updated motor output:

| Stage | Duration |
|---|---|
| ADC conversion and ISR entry | measured, `max_latency_us` |
| Control ISR | ≤ 72.9 µs (static bound) |
| Wait for the next PWM trough | ≤ 50 µs |

The SIL model is not changed by this work.

Commands run for this revision (output summarised; logs in the build directory):

| Check | Result |
|---|---|
| `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --warnings all firmware/src` | exit 0, 0 warnings in project files, 63,100 B flash, 7,164 B RAM |
| same, `--build-property "compiler.cpp.extra_flags=-DSG_TELEMETRY_SINK=3"` (USB + WiFi) | exit 0, 0 project warnings, 70,764 B flash, 9,436 B RAM |
| `arm_cycle_budget.py --elf` | control ISR PASS, 72.9 µs bound |
| `arm_cycle_budget.py` (probe mode, unchanged) | v2 step 33.6 µs, v3 worst 205.9 µs, 0 compiler warnings |
| `verify_control_theory.py` | 19/19 PASS |
| `run_pipeline.py` | PIPELINE PASSED. SHA-256 of all 156 files in `telemetry/data_sil/` and `telemetry/data_hil/` is identical before and after |

## 9. Not verified (requires the board)

- That `GTICASR` bit 16 latches on the ELC_GPT_A event as configured. The bit's position comes from the FSP `gpt_source_t` value, not from a named field in `R7FA4M1AB.h`.
- The polarity of the GTIOA = 0x03 triangle output, the 0 % behaviour with `GTCCRA` = 1201, and POEG forcing the output LOW.
- The ADC average-of-4 result scale in 14-bit mode against the v2 software average.
- The WDT timeout, and that no core code refreshes or reconfigures the WDT.
- That FspTimer assigns GPT4 to the LED matrix (printed at boot).
- The ESP32-S3 link pins, the level shifter, and the bridge sketch as a whole.
- Edge capture vs level sampling for contacts: v4 counts every falling edge on D7 (filtered at 4 µs, then gated by the unchanged 16 ms refractory window). v2 counted LOW levels seen at 2 ms sampling instants, so v4 can register short touches that v2 missed. Hardware contact counts are therefore not directly comparable with v2 SIL contact semantics until this is checked on the rig.
