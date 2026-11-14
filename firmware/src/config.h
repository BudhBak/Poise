#pragma once
// SymbioGrip configuration: Arduino UNO R4 WiFi (Renesas R7FA4M1AB)
// Pin facts verified against arduino:renesas_uno 1.6.0, variants/UNOWIFIR4/variant.cpp and pinmux.inc.

// ---------------------------------------------------------------------------------------------
// Pin map (UNO R4 WiFi; NOT the Minima map)
// ---------------------------------------------------------------------------------------------
#define PIN_ENC_B             2    // P104 -> GTIOC1B (GPT321 phase input B)
#define PIN_ENC_A             3    // P105 -> GTIOC1A (GPT321 phase input A)
#define PIN_TB_AIN1           4    // P106 -> TB6612FNG AIN1
#define PIN_TB_AIN2           5    // P107 -> TB6612FNG AIN2
#define PIN_TB_PWMA           6    // P111 -> GTIOC3A, PwmOut 20 kHz -> TB6612FNG PWMA
#define PIN_CONTACT           7    // P112 -> wire-loop contact sense, active LOW (INPUT_PULLUP + RC)
#define PIN_TB_STBY           8    // P304 -> TB6612FNG STBY (LOW = outputs disabled)
// OPAMP0 (unity-gain follower): A1 = P000 = AMP0+, A2 = P001 = AMP0-, A3 = P002 = AMP0O / AN002
#define PIN_GRIP_ADC          A3

// ---------------------------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------------------------
#define CONTROL_FREQ_HZ       500.0f
#define CONTROL_DT_SEC        0.002f
#define LOOP_PERIOD_US        2000UL

// ---------------------------------------------------------------------------------------------
// Encoder: LPD3806-600BM (600 PPR, NPN open collector) on the GT2 20T idler pulley (40 mm/rev)
// ---------------------------------------------------------------------------------------------
#define ENCODER_PPR           600
#define ENCODER_COUNTS_REV    (4 * ENCODER_PPR)                   // 4x quadrature = 2400
#define PULLEY_CIRCUM_M       0.040f                              // 20 teeth x 2 mm pitch
#define COUNTS_PER_METER      ((float)ENCODER_COUNTS_REV / PULLEY_CIRCUM_M)   // 60000
#define ENCODER_MAX_JUMP      200  // counts per tick (= 1.67 m/s); larger jumps are treated as faults

// ---------------------------------------------------------------------------------------------
// Damping law
// ---------------------------------------------------------------------------------------------
#define DAMPING_MIN           4.5f    // N*s/m, Mode C at rest
#define DAMPING_MAX           21.5f   // N*s/m, Mode C at full difficulty
#define DAMPING_FIXED         13.0f   // N*s/m, Mode B: pre-registered midpoint of the Mode C range
#define DAMPING_SLEW_ATTACK   240.0f  // (N*s/m) per second, rising
#define DAMPING_SLEW_RELEASE  55.0f   // (N*s/m) per second, falling
#define DIFF_W_GRIP           0.72f
#define DIFF_W_TREMOR         0.28f

// ---------------------------------------------------------------------------------------------
// Grip sensing: FSR 402 from 5 V to A1, R_M = 10 kOhm to GND, OPAMP0 follower, ADC at 14 bit.
// Divider model with FSR conductance ~ proportional to force: a = V/Vref = F / (F + S)
//   =>  F = S * a / (1 - a).   S [N] is calibrated with known weights (docs/CALIBRATION.md).
// ---------------------------------------------------------------------------------------------
#define ADC_BITS              14
#define ADC_FULL_SCALE        16383.0f
#define ADC_OVERSAMPLE        4
#define FSR_FORCE_SCALE_N     1.0f
#define GRIP_REST_N           2.5f    // force below which grip is "relaxed"
#define GRIP_FULL_N           8.0f    // force mapped to normalized grip = 1
#define GRIP_LP_ALPHA         0.0951626f   // 1 - exp(-T/20 ms)

// ---------------------------------------------------------------------------------------------
// Tremor estimator (5-25 Hz band of carriage acceleration)
// Quantization noise floor at 60000 counts/m, 500 Hz: sigma = 0.062-0.075 m/s^2
// (verify_control_theory.py recomputes this; floor must be >= 3 sigma).
// ---------------------------------------------------------------------------------------------
#define TREMOR_MS_ALPHA       0.00796809f  // 1 - exp(-T/250 ms), mean-square EMA
#define TREMOR_RMS_FLOOR      0.25f   // m/s^2 RMS
#define TREMOR_RMS_SPAN       0.50f   // m/s^2 RMS from floor to full scale

// ---------------------------------------------------------------------------------------------
// Actuator: 12 V brushed DC gearmotor, 4.4:1 (e.g. Pololu 37D 4.4:1), GT2 20T pulley r = 6.366 mm,
// driven by TB6612FNG (1.2 A continuous). Values are datasheet-derived defaults; measure yours
// (docs/CALIBRATION.md). KT_EFF is the force constant AT THE CARRIAGE (N/A) = Kt * N / r.
// ---------------------------------------------------------------------------------------------
#define MOTOR_KT_EFF          5.6f    // N/A  (also back-EMF constant in V/(m/s), SI identity)
#define MOTOR_R_OHM           2.2f    // terminal resistance
#define MOTOR_SUPPLY_V        12.0f
#define DRIVER_I_MAX          1.2f    // A, TB6612FNG continuous rating
#define BEMF_COMPENSATION     1.0f    // fraction of back-EMF cancelled in software
#define MOTOR_POLARITY_INVERT 0       // set to 1 if the 'P' polarity test reports INVERTED
#define VEL_DEADBAND          0.004f  // m/s: below this the bridge coasts (no quantization chatter)
#define VEL_FAULT             0.50f   // m/s: above this the encoder is assumed faulty -> coast + latch

// ---------------------------------------------------------------------------------------------
// Time-domain passivity (observer + controller)
// ---------------------------------------------------------------------------------------------
#define PC_BETA_MAX           10.0f   // N*s/m, max extra damping added by the passivity controller
#define PO_ENERGY_CAP_J       0.10f   // J, bound on stored dissipation budget
#define B_FRICTION_EST        1.0f    // N*s/m, rail + gearbox viscous friction (for verification only)

// ---------------------------------------------------------------------------------------------
// Task / logging
// ---------------------------------------------------------------------------------------------
#define CONTACT_REFRACTORY_TICKS 8    // 16 ms, identical in firmware, SIL and desktop bench
#define TELEMETRY_BAUD        115200UL
#define TELEMETRY_DIV         20      // 500 Hz / 20 = 25 Hz rows
#define TX_BYTES_PER_TICK     5       // worst-case blocking ~ 4 x 86.8 us = 0.35 ms at 115200
#define TX_QUEUE_BYTES        512
#define TELEMETRY_ROW_MAX     96      // hard buffer limit per row
#define TELEMETRY_ROW_TYPICAL 72      // measured length of a typical row

// Throughput sanity: drip capacity (B/s) must exceed typical row demand (B/s).
static_assert((TX_BYTES_PER_TICK * 500) >= (TELEMETRY_ROW_TYPICAL * (500 / TELEMETRY_DIV)),
              "telemetry drip rate too low for the configured row rate");

// ---------------------------------------------------------------------------------------------
// Hardware fabric (docs/HardwareFabric.md). Firmware only: the simulation never uses these.
// Clock tree (variants/UNOWIFIR4/includes/ra_gen/bsp_clock_cfg.h): HOCO 48 MHz, PCLKD = PCLKC = 48 MHz,
// PCLKB = 24 MHz.
// ---------------------------------------------------------------------------------------------
#define HW_PCLKD_HZ           48000000UL   // GPT count clock
#define HW_PCLKB_HZ           24000000UL   // WDT clock
#define HW_PACER_GPT_CH       0            // GPT320: 500 Hz pacing timer, no pin output
#define HW_PACER_PERIOD       (HW_PCLKD_HZ / 500UL)          // 96000 counts = 2000 us
#define HW_PWM_GPT_CH         3            // GPT163: GTIOC3A = P111 (D6) PWM out, GTIOC3B = P112 (D7) capture in
#define HW_PWM_FREQ_HZ        20000UL
#define HW_PWM_GTPR           (HW_PCLKD_HZ / (2UL * HW_PWM_FREQ_HZ))   // 1200: symmetric triangle, period 2*GTPR
#define HW_ENC_GPT_CH         1            // GPT321: 4x phase counting on GTIOC1A/B = P105/P104
#define HW_CONTROL_IRQ_PRIO   1            // NVIC 0..15 (__NVIC_PRIO_BITS = 4); AGT0 millis = 8, LED matrix = 12
#define HW_ADC_CH_GRIP        2            // AN002 = P002 = OPAMP0 output
#define HW_ADC_CH_AMP_IN      0            // AN000 = P000 = OPAMP0 non-inverting input (self-test only)
#define HW_ADC_CH_DAC_LOOP    9            // AN009 = P014 = DAC12 output pin A0 (self-test loopback only)
#define HW_ADC_SAMPLE_STATES  48           // ADSSTR02: 1.0 us at PCLKC 48 MHz (driven by the op-amp output)
#define HW_ADC_SELFTEST_STATES 255         // 5.3 us for the high-impedance DAC pin and internal reference
#define HW_ADADC_AVG4         0x83         // AVEE = 1, ADC = 3: average of 4 (FSP ADC_ADD_AVERAGE_FOUR)
#define HW_ADSTRGR_ELC        0x093F       // TRSA = 0x09 ELC_AD00 (FSP adc_elc_trigger_lut), TRSB = 0x3F off
#define HW_WDTCR              0x3341       // TOPS 4096 cyc, CKS PCLKB/64, window 0..100 % -> 10.92 ms timeout
#define HW_WDT_TIMEOUT_US     10923UL
static_assert(HW_WDT_TIMEOUT_US > 4UL * LOOP_PERIOD_US, "WDT must tolerate >= 4 missed-free control periods");
static_assert(HW_PACER_PERIOD % (2UL * HW_PWM_GTPR) == 0, "PWM period must divide the pacer period (ADC at PWM valley)");
#define HW_MAX_ISR_US_BUDGET  250UL        // control ISR must finish within this (static bound + margin)
#define HW_PWM_MIN_COUNTS     2            // on-counts below this -> output held LOW; above GTPR-this -> 99.8 % max

// ---------------------------------------------------------------------------------------------
// Telemetry sinks: 1 = USB (Serial -> ESP32-S3 USB bridge), 2 = WiFi UDP via the stock ESP32-S3
// firmware (WiFiS3, Serial2 AT link), 3 = both. WiFi credentials come from secrets.h (git-ignored).
// ---------------------------------------------------------------------------------------------
#ifndef SG_TELEMETRY_SINK
#define SG_TELEMETRY_SINK     1
#endif
#define TELEMETRY_UDP_PORT    5005
#define TELEMETRY_UDP_BATCH   5            // rows per UDP datagram (5 rows = 200 ms at 25 Hz)
