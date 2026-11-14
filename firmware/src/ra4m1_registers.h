#pragma once
// RA4M1 (R7FA4M1AB) peripheral fabric for SymbioGrip on the Arduino UNO R4 WiFi.
// Design and citations: docs/HardwareFabric.md. Every register position used here was checked
// against arduino:renesas_uno 1.6.0 (R7FA4M1AB.h bitfield _Pos macros, bsp_module_stop.h, bsp_elc.h,
// r_ioport_api.h, variant.cpp, pinmux.inc) and the FSP objects shipped in libfsp.a (r_adc.o, r_gpt.o).
//
// Event chain (no CPU involvement until the control ISR):
//   GPT0 overflow (500 Hz, ELC event 93)
//     -> ELSR0 (ELC_GPT_A) -> GPT1 GTICASR bit 16: encoder GTCNT latched into GPT1.GTCCRA
//     -> ELSR8 (ELC_AD00)  -> ADC14 single scan of AN002 (4x hardware average)
//   ADC0_SCAN_END (ELC event 41) -> NVIC priority 1 -> control ISR (src.ino)
//   GPT3: 20 kHz triangle PWM on GTIOC3A (P111), started in the same GTSTR write as GPT0, so every
//         pacer overflow is a PWM trough (centre of the off-time). GTIOC3B (P112) captures contact edges.
//   POEG group A: software/latched output disable of GTIOC3A (forced LOW).  WDT: reset if the ISR stops.
#include <Arduino.h>
#include <OPAMP.h>
#include "config.h"

namespace RA4M1_Hw {

// ---------------------------------------------------------------------------------------------
// Encoder (GPT321, unchanged from v2)
// ---------------------------------------------------------------------------------------------
static const uint32_t ENC_OFFSET = 0x80000000UL;   // signed position = GTCNT - offset

// 4x quadrature, A leads B = count up:
//   up:   A rise & B low (b8) | A fall & B high (b11) | B rise & A high (b13) | B fall & A low (b14)
//   down: A rise & B high (b9) | A fall & B low (b10) | B rise & A low (b12) | B fall & A high (b15)
static const uint32_t GTUPSR_4X = (1UL << R_GPT0_GTUPSR_USCARBL_Pos) | (1UL << R_GPT0_GTUPSR_USCAFBH_Pos) |
                                  (1UL << R_GPT0_GTUPSR_USCBRAH_Pos) | (1UL << R_GPT0_GTUPSR_USCBFAL_Pos);
static const uint32_t GTDNSR_4X = (1UL << R_GPT0_GTUPSR_USCARBH_Pos) | (1UL << R_GPT0_GTUPSR_USCAFBL_Pos) |
                                  (1UL << R_GPT0_GTUPSR_USCBRAL_Pos) | (1UL << R_GPT0_GTUPSR_USCBFAH_Pos);
static_assert(GTUPSR_4X == 0x6900UL, "GTUPSR 4x pattern");
static_assert(GTDNSR_4X == 0x9600UL, "GTDNSR 4x pattern");

// Digital noise filter on both phase inputs, PCLKD/64 (NFCSx = 3): 3 x 1.33 us = 4 us stable.
static const uint32_t GTIOR_NF = (1UL << R_GPT0_GTIOR_NFAEN_Pos) | (3UL << R_GPT0_GTIOR_NFCSA_Pos) |
                                 (1UL << R_GPT0_GTIOR_NFBEN_Pos) | (3UL << R_GPT0_GTIOR_NFCSB_Pos);

// ELC event inputs of a GPT channel: ELC_GPT_A..H map to source-select bits 16..23
// (FSP gpt_source_t GPT_SOURCE_GPT_A = 1 << 16; R7FA4M1AB.h names bit 16 of GTICASR "ASELCA").
static const uint32_t GPT_SRC_ELC_GPT_A = (1UL << 16);

inline void initHardwareQuadratureEncoder() {
    FspTimer::set_timer_is_used(GPT_TIMER, HW_ENC_GPT_CH);
    R_BSP_MODULE_START(FSP_IP_GPT, 1);            // MSTPCRD.MSTPD5 = 0 (GPT320/GPT321)

    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_05,
                    (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));   // GTIOC1A
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_04,
                    (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));   // GTIOC1B

    R_GPT1->GTSTP   = (1UL << 1);
    R_GPT1->GTCR    = 0;                          // saw-wave, PCLKD/1, stopped
    R_GPT1->GTUPSR  = GTUPSR_4X;
    R_GPT1->GTDNSR  = GTDNSR_4X;
    R_GPT1->GTIOR   = GTIOR_NF;
    R_GPT1->GTICASR = GPT_SRC_ELC_GPT_A;          // capture GTCNT -> GTCCRA on the pacer event
    R_GPT1->GTPR    = 0xFFFFFFFFUL;
    R_GPT1->GTCNT   = ENC_OFFSET;
    R_GPT1->GTSTR   = (1UL << 1);
}

inline int32_t readHardwareEncoder() {            // live counter
    return (int32_t)(R_GPT1->GTCNT - ENC_OFFSET);
}

inline int32_t readLatchedEncoder() {             // value latched by the pacer event
    return (int32_t)(R_GPT1->GTCCR[0] - ENC_OFFSET);
}

inline void resetHardwareEncoder() {
    R_GPT1->GTSTP = (1UL << 1);
    R_GPT1->GTCNT = ENC_OFFSET;
    R_GPT1->GTSTR = (1UL << 1);
}

// ---------------------------------------------------------------------------------------------
// OPAMP0 follower (A1 = AMP0+, A2 = AMP0- tied to A3 = AMP0O)
// ---------------------------------------------------------------------------------------------
inline bool initInternalOpamp() {
    R_BSP_MODULE_START(FSP_IP_OPAMP, 0);          // MSTPCRD.MSTPD31 = 0
    return OPAMP.begin(OPAMP_SPEED_HIGHSPEED);
}

// ---------------------------------------------------------------------------------------------
// ADC14, programmed directly. analogRead() must never be called after this: the core re-opens the
// ADC through FSP on every call and would overwrite ADSTRGR/ADANSA.
// ---------------------------------------------------------------------------------------------
static const uint16_t ADCSR_ADST = (1U << R_ADC0_ADCSR_ADST_Pos);   // bit 15
static const uint16_t ADCSR_ADIE = (1U << R_ADC0_ADCSR_ADIE_Pos);   // bit 12
static const uint16_t ADCSR_TRGE = (1U << R_ADC0_ADCSR_TRGE_Pos);   // bit 9
static const uint16_t ADCER_14BIT = (3U << R_ADC0_ADCER_ADPRC_Pos); // ADPRC = 3, right-aligned
static const uint16_t ADEXICR_OCSA = (1U << R_ADC0_ADEXICR_OCSA_Pos);  // bit 9
static const uint16_t ADEXICR_TSSA = (1U << R_ADC0_ADEXICR_TSSA_Pos);  // bit 8

inline void adcPinAnalog(bsp_io_port_pin_t pin) {
    R_IOPORT_PinCfg(&g_ioport_ctrl, pin, (uint32_t)IOPORT_CFG_ANALOG_ENABLE);  // PmnPFS.ASEL = 1
}

inline void initAdcBase() {
    R_BSP_MODULE_START(FSP_IP_ADC, 0);            // MSTPCRD.MSTPD16 = 0
    R_ADC0->ADCSR   = 0;                          // single scan, software trigger, stopped
    R_ADC0->ADCER   = ADCER_14BIT;
    R_ADC0->ADADC   = 0;
    R_ADC0->ADEXICR = 0;
    R_ADC0->ADANSA[0] = 0;
    R_ADC0->ADANSA[1] = 0;
    R_ADC0->ADSSTR[HW_ADC_CH_GRIP]     = HW_ADC_SAMPLE_STATES;
    R_ADC0->ADSSTR[HW_ADC_CH_AMP_IN]   = HW_ADC_SELFTEST_STATES;
    R_ADC0->ADSSTR[HW_ADC_CH_DAC_LOOP] = HW_ADC_SELFTEST_STATES;
    R_ADC0->ADSSTRO = HW_ADC_SELFTEST_STATES;     // internal reference
    R_ADC0->ADSSTRT = HW_ADC_SELFTEST_STATES;     // temperature sensor
    adcPinAnalog(BSP_IO_PORT_00_PIN_02);          // AN002
}

// Blocking software-triggered conversion, boot self-test only (before the pipeline starts).
// ext = 0 for a pin channel, or ADEXICR_OCSA / ADEXICR_TSSA with no pin channel selected.
inline bool adcSoftwareConvert(int ch, uint16_t ext, uint16_t avg, uint16_t& out) {
    R_ADC0->ADCSR     = 0;
    R_ADC0->ADANSA[0] = (ch >= 0) ? (uint16_t)(1U << ch) : 0;
    R_ADC0->ADEXICR   = ext;
    R_ADC0->ADADC     = (uint8_t)avg;
    R_ADC0->ADCSR     = ADCSR_ADST;
    const uint32_t t0 = micros();
    while (R_ADC0->ADCSR & ADCSR_ADST) {
        if ((uint32_t)(micros() - t0) > 1000UL) return false;
    }
    if (ext == ADEXICR_OCSA)      out = R_ADC0->ADOCDR;
    else if (ext == ADEXICR_TSSA) out = R_ADC0->ADTSDR;
    else                          out = R_ADC0->ADDR[ch];
    return true;
}

// Switch the ADC to the hardware pipeline: AN002 only, 4x average, started by ELC_AD00.
inline void armAdcPipeline() {
    R_ADC0->ADCSR     = 0;
    R_ADC0->ADEXICR   = 0;
    R_ADC0->ADANSA[0] = (uint16_t)(1U << HW_ADC_CH_GRIP);
    R_ADC0->ADANSA[1] = 0;
    R_ADC0->ADADC     = HW_ADADC_AVG4;
    R_ADC0->ADSTRGR   = HW_ADSTRGR_ELC;
    R_ADC0->ADCSR     = (uint16_t)(ADCSR_TRGE | ADCSR_ADIE);   // EXTRG = 0: synchronous (ELC) trigger
}

inline uint16_t readGripResult() { return R_ADC0->ADDR[HW_ADC_CH_GRIP]; }

// ---------------------------------------------------------------------------------------------
// GPT0 pacer and ELC links
// ---------------------------------------------------------------------------------------------
inline void initPacer() {
    FspTimer::set_timer_is_used(GPT_TIMER, HW_PACER_GPT_CH);
    R_BSP_MODULE_START(FSP_IP_GPT, 0);
    R_GPT0->GTSTP = (1UL << 0);
    R_GPT0->GTCR  = 0;                            // saw-wave up-count, PCLKD/1
    R_GPT0->GTUDDTYC = 1;                         // UD = 1 (up)
    R_GPT0->GTIOR = 0;                            // no pin
    R_GPT0->GTPR  = HW_PACER_PERIOD - 1UL;        // 95999
    R_GPT0->GTCNT = 0;
}

inline void initElcLinks() {
    R_BSP_MODULE_START(FSP_IP_ELC, 0);            // MSTPCRC.MSTPC14 = 0
    R_ELC->ELSR[ELC_PERIPHERAL_GPT_A].HA = (uint16_t)ELC_EVENT_GPT0_COUNTER_OVERFLOW;  // ELSR0 = 93
    R_ELC->ELSR[ELC_PERIPHERAL_ADC0].HA  = (uint16_t)ELC_EVENT_GPT0_COUNTER_OVERFLOW;  // ELSR8 = 93
    R_ELC->ELCR = (uint8_t)(1U << R_ELC_ELCR_ELCON_Pos);
}

inline uint32_t pacerCount() { return R_GPT0->GTCNT; }

// ---------------------------------------------------------------------------------------------
// GPT3: 20 kHz symmetric triangle PWM on GTIOC3A (P111) + contact capture on GTIOC3B (P112)
// GTIOA = 0x03: toggle at GTCCRA compare match, retain at cycle end, initial LOW (FSP
// GPT_PRV_GTIO_TOGGLE_COMPARE_MATCH). Output is HIGH while GTCNT > GTCCRA, so
//   on-counts = GTPR - GTCCRA, and the LOW interval is centred on the trough (GTCNT = 0).
// GTCCRA must stay in [1, GTPR-1] (two matches per period) or > GTPR (no match, output stays LOW);
// GTCCRA = 0 or = GTPR gives a single match per period and would invert the toggle phase.
// ---------------------------------------------------------------------------------------------
static const uint32_t GTCR_TRIANGLE_1 = (4UL << R_GPT0_GTCR_MD_Pos);          // MD = 100b, transfer at trough
static const uint32_t GTIOR_PWM = 0x03UL                                      // GTIOA
                                | (1UL << R_GPT0_GTIOR_OAE_Pos)               // GTIOC3A output enable
                                | (2UL << R_GPT0_GTIOR_OADF_Pos)              // output-disable level: LOW
                                | (1UL << R_GPT0_GTIOR_NFBEN_Pos)             // contact input filter
                                | (3UL << R_GPT0_GTIOR_NFCSB_Pos);            // PCLKD/64 (4 us)
static const uint32_t GTICBSR_CONTACT = (1UL << R_GPT0_GTICBSR_BSCBFAL_Pos) | // GTIOC3B falling edge,
                                        (1UL << R_GPT0_GTICBSR_BSCBFAH_Pos);  // either GTIOC3A level
static const uint32_t GTBER_CCRA_SINGLE = (1UL << R_GPT0_GTBER_CCRA_Pos);
static const uint32_t PWM_OFF_CCRA = HW_PWM_GTPR + 1UL;                       // never matches -> LOW
static const uint32_t GTST_TCFB = (1UL << R_GPT0_GTST_TCFB_Pos);

inline void initPwmAndContact() {
    FspTimer::set_timer_is_used(GPT_TIMER, HW_PWM_GPT_CH);
    R_BSP_MODULE_START(FSP_IP_GPT, 3);            // MSTPCRD.MSTPD6 = 0 (GPT162..GPT167)
    R_GPT3->GTSTP  = (1UL << 3);
    R_GPT3->GTCR   = GTCR_TRIANGLE_1;
    R_GPT3->GTUDDTYC = 1;
    R_GPT3->GTPR   = HW_PWM_GTPR;
    R_GPT3->GTCNT  = 0;
    R_GPT3->GTBER  = GTBER_CCRA_SINGLE;           // GTCCRC -> GTCCRA at each trough
    R_GPT3->GTCCR[0] = PWM_OFF_CCRA;              // GTCCRA
    R_GPT3->GTCCR[2] = PWM_OFF_CCRA;              // GTCCRC (buffer)
    R_GPT3->GTICBSR = GTICBSR_CONTACT;
    R_GPT3->GTINTAD = 0;                          // GRP = 0: POEG group A; no GPT interrupts
    R_GPT3->GTIOR  = GTIOR_PWM;
    R_GPT3->GTST   = 0;
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_11,
                    (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));   // GTIOC3A
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_12,
                    (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1 |
                               IOPORT_CFG_PULLUP_ENABLE));                             // GTIOC3B + pull-up
}

// |duty| in [0, 1] -> buffered compare value. Takes effect at the next PWM trough.
inline void setPwmDuty(float duty_abs) {
    uint32_t on = (uint32_t)(duty_abs * (float)HW_PWM_GTPR + 0.5f);
    uint32_t ccra;
    if (on < (uint32_t)HW_PWM_MIN_COUNTS)                 ccra = PWM_OFF_CCRA;
    else if (on > HW_PWM_GTPR - HW_PWM_MIN_COUNTS)        ccra = HW_PWM_MIN_COUNTS;   // 99.8 % max
    else                                                  ccra = HW_PWM_GTPR - on;
    R_GPT3->GTCCR[2] = ccra;
}

// True if at least one falling edge reached GTIOC3B since the previous call.
inline bool takeContactEdge() {
    if (R_GPT3->GTST & GTST_TCFB) {
        R_GPT3->GTST = ~GTST_TCFB;                // flags clear on write 0, writes of 1 are ignored
        return true;
    }
    return false;
}

// Start GPT0 and GPT3 in one write: both counters start on the same PCLKD edge.
inline void startPacerAndPwm() {
    R_GPT0->GTSTR = (1UL << HW_PACER_GPT_CH) | (1UL << HW_PWM_GPT_CH);
}

// ---------------------------------------------------------------------------------------------
// Direction / standby pins through the atomic set/reset register PCNTR3 (POSR 0..15, PORR 16..31)
// AIN1 = P106, AIN2 = P107, STBY = P304.
// ---------------------------------------------------------------------------------------------
static const uint32_t PORT1_AIN1 = (1UL << 6), PORT1_AIN2 = (1UL << 7), PORT3_STBY = (1UL << 4);

inline void bridgeDirection(int dir) {            // +1: AIN1 H / AIN2 L, -1: L / H, 0: L / L (stop)
    if (dir > 0)      R_PORT1->PCNTR3 = PORT1_AIN1 | (PORT1_AIN2 << 16);
    else if (dir < 0) R_PORT1->PCNTR3 = PORT1_AIN2 | (PORT1_AIN1 << 16);
    else              R_PORT1->PCNTR3 = (PORT1_AIN1 | PORT1_AIN2) << 16;
}

inline void bridgeStandby(bool enabled) {
    R_PORT3->PCNTR3 = enabled ? PORT3_STBY : (PORT3_STBY << 16);
}

// ---------------------------------------------------------------------------------------------
// POEG group A: software output disable of GTIOC3A (latched until reset in this firmware).
// ---------------------------------------------------------------------------------------------
static const uint32_t POEGG_SSF = (1UL << R_GPT_POEG0_POEGG_SSF_Pos);   // bit 3

inline void initPoeg() {
    R_BSP_MODULE_START(FSP_IP_POEG, 0);           // MSTPCRD.MSTPD14 = 0
}

inline void poegTrip()       { R_GPT_POEG0->POEGG = POEGG_SSF; }
inline bool poegTripped()    { return (R_GPT3->GTST & (1UL << R_GPT0_GTST_ODF_Pos)) != 0; }

// ---------------------------------------------------------------------------------------------
// WDT, register-start mode (OFS0 in the core = 0xFFFFFFFF). WDTCR is writable once after reset.
// ---------------------------------------------------------------------------------------------
inline void startWatchdog() {
    R_WDT->WDTCR  = HW_WDTCR;
    R_WDT->WDTRCR = (uint8_t)(1U << R_WDT_WDTRCR_RSTIRQS_Pos);   // underflow / refresh error -> reset
    R_WDT->WDTRR  = 0x00;
    R_WDT->WDTRR  = 0xFF;                         // first refresh starts the counter
}

inline void kickWatchdog() {
    R_WDT->WDTRR = 0x00;
    R_WDT->WDTRR = 0xFF;
}

// ---------------------------------------------------------------------------------------------
// Self-test helpers
// ---------------------------------------------------------------------------------------------
inline bool checkLine(Print& out, const char* name, uint32_t value, bool pass) {
    out.print(F("#SELFTEST,"));
    out.print(name);
    out.print(F(",0x"));
    out.print(value, HEX);
    out.println(pass ? F(",PASS") : F(",FAIL"));
    return pass;
}

inline uint32_t pfs(uint8_t port, uint8_t pin) { return R_PFS->PORT[port].PIN[pin].PmnPFS; }

// Register read-back. Safe to call while the pipeline runs (reads only).
inline bool runRegisterSelfTest(Print& out) {
    bool ok = true;
    const uint32_t mstpd = R_MSTP->MSTPCRD;
    ok &= checkLine(out, "MSTPCRD.D5(GPT0-1)|D6(GPT2-7)|D14(POEG)|D16(ADC)=0", mstpd,
                    (mstpd & ((1UL << 5) | (1UL << 6) | (1UL << 14) | (1UL << 16))) == 0);
    ok &= checkLine(out, "MSTPCRD.D31(OPAMP)=0", mstpd, (mstpd & (1UL << 31)) == 0);
    ok &= checkLine(out, "MSTPCRC.C14(ELC)=0", R_MSTP->MSTPCRC, (R_MSTP->MSTPCRC & (1UL << 14)) == 0);

    const uint32_t pfs_mask = (0x1FUL << R_PFS_PORT_PIN_PmnPFS_PSEL_Pos) | (1UL << R_PFS_PORT_PIN_PmnPFS_PMR_Pos);
    const uint32_t pfs_gpt  = (3UL << R_PFS_PORT_PIN_PmnPFS_PSEL_Pos) | (1UL << R_PFS_PORT_PIN_PmnPFS_PMR_Pos);
    ok &= checkLine(out, "P105.PFS=GTIOC1A", pfs(1, 5), (pfs(1, 5) & pfs_mask) == pfs_gpt);
    ok &= checkLine(out, "P104.PFS=GTIOC1B", pfs(1, 4), (pfs(1, 4) & pfs_mask) == pfs_gpt);
    ok &= checkLine(out, "P111.PFS=GTIOC3A", pfs(1, 11), (pfs(1, 11) & pfs_mask) == pfs_gpt);
    ok &= checkLine(out, "P112.PFS=GTIOC3B+PCR", pfs(1, 12),
                    (pfs(1, 12) & pfs_mask) == pfs_gpt && (pfs(1, 12) & (1UL << R_PFS_PORT_PIN_PmnPFS_PCR_Pos)));
    ok &= checkLine(out, "P002.PFS.ASEL(AN002)", pfs(0, 2), (pfs(0, 2) & (1UL << R_PFS_PORT_PIN_PmnPFS_ASEL_Pos)) != 0);

    ok &= checkLine(out, "GPT1.GTUPSR=0x6900", R_GPT1->GTUPSR, R_GPT1->GTUPSR == GTUPSR_4X);
    ok &= checkLine(out, "GPT1.GTDNSR=0x9600", R_GPT1->GTDNSR, R_GPT1->GTDNSR == GTDNSR_4X);
    ok &= checkLine(out, "GPT1.GTICASR=ELC_GPT_A", R_GPT1->GTICASR, R_GPT1->GTICASR == GPT_SRC_ELC_GPT_A);
    ok &= checkLine(out, "GPT0.GTPR=95999", R_GPT0->GTPR, R_GPT0->GTPR == HW_PACER_PERIOD - 1UL);
    ok &= checkLine(out, "GPT3.GTCR.MD=4", R_GPT3->GTCR, (R_GPT3->GTCR & (7UL << R_GPT0_GTCR_MD_Pos)) == GTCR_TRIANGLE_1);
    ok &= checkLine(out, "GPT3.GTPR=1200", R_GPT3->GTPR, R_GPT3->GTPR == HW_PWM_GTPR);
    ok &= checkLine(out, "GPT3.GTIOR", R_GPT3->GTIOR, R_GPT3->GTIOR == GTIOR_PWM);
    ok &= checkLine(out, "ELC.ELSR0=93", R_ELC->ELSR[0].HA, R_ELC->ELSR[0].HA == ELC_EVENT_GPT0_COUNTER_OVERFLOW);
    ok &= checkLine(out, "ELC.ELSR8=93", R_ELC->ELSR[8].HA, R_ELC->ELSR[8].HA == ELC_EVENT_GPT0_COUNTER_OVERFLOW);
    ok &= checkLine(out, "OPAMP.AMPMON0", R_OPAMP->AMPMON, OPAMP.isRunning(0));
    // Informational: FspTimer should have given the LED matrix (10 kHz) the first free GPT, GPT4.
    out.print(F("#SELFTEST,GPT4.GTPR(LED_matrix_expect_4799),0x")); out.print(R_GPT4->GTPR, HEX);
    out.println(R_GPT4->GTPR == 4799UL ? F(",INFO_MATCH") : F(",INFO_MISMATCH"));
    return ok;
}

// Boot analog self-test (software-triggered; must run before armAdcPipeline()).
//  1. Internal reference (1.43 V, pins_arduino.h AR_INTERNAL_VOLTAGE) -> inferred AVCC0 in [4.5, 5.5] V.
//  2. DAC12 (A0 = P014) -> AN009 on the same pin, codes 1024/2048/3072: ADC = 4 x DAC within 2 % FS.
//     A0 must be left unconnected.
//  3. OPAMP0 follower: AN002 (output) vs AN000 (input) within 2 % FS.
//  4. Temperature sensor: raw ADTSDR and the factory calibration word, informational only.
inline bool runAnalogSelfTest(Print& out, float& avcc_out) {
    bool ok = true;
    uint16_t v = 0;
    avcc_out = 0.0f;

    bool conv = adcSoftwareConvert(-1, ADEXICR_OCSA, 0, v);
    const float avcc = (conv && v > 0) ? (1.43f * ADC_FULL_SCALE / (float)v) : 0.0f;
    avcc_out = avcc;
    ok &= checkLine(out, "ADC.VREFINT->AVCC_mV", (uint32_t)(avcc * 1000.0f), conv && avcc > 4.5f && avcc < 5.5f);

    adcPinAnalog(BSP_IO_PORT_00_PIN_14);
    analogWriteResolution(12);
    const uint16_t codes[3] = {1024, 2048, 3072};
    for (int i = 0; i < 3; ++i) {
        analogWrite(DAC, codes[i]);
        delayMicroseconds(200);
        conv = adcSoftwareConvert(HW_ADC_CH_DAC_LOOP, 0, HW_ADADC_AVG4, v);
        const int32_t err = (int32_t)v - 4 * (int32_t)codes[i];
        ok &= checkLine(out, "DAC12->AN009", v, conv && abs(err) <= (int32_t)(0.02f * ADC_FULL_SCALE));
    }
    analogWrite(DAC, 0);

    adcPinAnalog(BSP_IO_PORT_00_PIN_00);
    uint16_t vin = 0, vout = 0;
    conv  = adcSoftwareConvert(HW_ADC_CH_AMP_IN, 0, HW_ADADC_AVG4, vin);
    conv &= adcSoftwareConvert(HW_ADC_CH_GRIP, 0, HW_ADADC_AVG4, vout);
    ok &= checkLine(out, "OPAMP0.out-in", (uint32_t)abs((int32_t)vout - (int32_t)vin),
                    conv && abs((int32_t)vout - (int32_t)vin) <= (int32_t)(0.02f * ADC_FULL_SCALE));

    R_BSP_MODULE_START(FSP_IP_TSN, 0);
    delayMicroseconds(50);
    conv = adcSoftwareConvert(-1, ADEXICR_TSSA, 0, v);
    const uint32_t cal = ((uint32_t)R_TSN->TSCDRH << 8) | R_TSN->TSCDRL;
    out.print(F("#SELFTEST,TSN.raw,0x")); out.print(v, HEX);
    out.print(F(",cal,0x")); out.print(cal, HEX);
    out.println(conv ? F(",INFO") : F(",TIMEOUT"));
    return ok;
}

}  // namespace RA4M1_Hw
