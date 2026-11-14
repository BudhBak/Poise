// SymbioGrip: adaptive haptic hand stabilizer, Arduino UNO R4 WiFi (Renesas RA4M1)
//
// Hardware-paced pipeline (docs/HardwareFabric.md):
//   GPT0 overflow (500 Hz) --ELC--> GPT1 encoder capture + ADC14 grip scan (4x average)
//   ADC scan end --> control ISR (NVIC priority 1): HapticEngine::step (admittance.h, unchanged)
//   --> GPT3 20 kHz triangle PWM (buffered, applied at the next trough) + AIN1/AIN2 via PCNTR3.
//   loop() only formats telemetry, parses commands and drives the LED pacer; it may block.
//
// Serial commands (115200 baud):
//   A / B / C  select condition (unassisted / fixed 13 N*s/m / adaptive); resets encoder + engine
//   R          reset encoder + engine + contact count, keep mode
//   P          actuator polarity test (carriage must be free to move ~5 mm)
//   S          re-run register self-test (read-only)
//
// Telemetry header (25 Hz):  t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts
// Status (1 Hz):             #STAT,t_ms,overruns,drops,max_isr_us,pc_activations,fault,armed,max_latency_us,poeg

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"
#include "IRQManager.h"
#include "config.h"
#include "ra4m1_registers.h"
#include "admittance.h"
#include "fabric_types.h"

#if (SG_TELEMETRY_SINK & 2)
#include <WiFiS3.h>
#include <WiFiUdp.h>
#if __has_include("secrets.h")
#include "secrets.h"            // defines SECRET_SSID, SECRET_PASS, optional SECRET_UDP_HOST
#endif
#ifndef SECRET_SSID
#define SECRET_SSID ""
#define SECRET_PASS ""
#endif
#ifndef SECRET_UDP_HOST
#define SECRET_UDP_HOST "255.255.255.255"
#endif
static WiFiUDP udp;
static bool wifi_ok = false;
#endif

static HapticEngine engine;
static ArduinoLEDMatrix matrix;

// ---------------------------------------------------------------------------------------------
// State shared between the control ISR and loop()
// ---------------------------------------------------------------------------------------------

static Snapshot snap;
static volatile uint32_t snap_seq = 0;            // odd while the ISR is writing

static volatile uint32_t isr_ticks = 0;
static volatile uint32_t overruns = 0;
static volatile uint32_t max_isr_counts = 0;      // PCLKD counts (48 per us)
static volatile uint32_t max_latency_counts = 0;  // pacer overflow -> ISR entry
static volatile char cmd_request = 0;             // written by loop(), consumed by the ISR
static volatile int32_t polarity_result = 0;      // 0 = none pending, else counts moved (+1 bias)
static volatile bool polarity_done = false;

static int32_t contacts = 0;
static uint8_t contact_refractory = 0;
static bool selftest_ok = false;
static volatile bool polarity_ok = false;
static int last_dir = 0;

static GenericIrqCfg_t control_irq = {FSP_INVALID_VECTOR, HW_CONTROL_IRQ_PRIO, ELC_EVENT_ADC0_SCAN_END};

// Polarity test state machine (runs inside the ISR): 75 ticks at 20 % duty, 75 ticks coast.
static uint16_t polarity_ticks = 0;
static const uint16_t POLARITY_DRIVE_TICKS = 75, POLARITY_TOTAL_TICKS = 150;

static void driveBridge(float duty) {             // signed duty, armed path only
#if MOTOR_POLARITY_INVERT
    duty = -duty;
#endif
    const int dir = duty >= 0.0f ? 1 : -1;
    if (dir != last_dir) {                        // break-before-make: one tick with the bridge stopped
        RA4M1_Hw::setPwmDuty(0.0f);
        RA4M1_Hw::bridgeDirection(0);
        last_dir = dir;
        return;
    }
    RA4M1_Hw::bridgeDirection(dir);
    RA4M1_Hw::setPwmDuty(fabsf(duty));
}

static void coastBridge() {
    RA4M1_Hw::setPwmDuty(0.0f);
    RA4M1_Hw::bridgeDirection(0);                 // IN1 = IN2 = L -> outputs OFF (high impedance)
    last_dir = 0;
}

static void handleRequest(char c) {
    if (c == 'A' || c == 'B' || c == 'C') {
        RA4M1_Hw::resetHardwareEncoder();
        engine.setMode(c, RA4M1_Hw::readHardwareEncoder());
        contacts = 0;
        contact_refractory = 0;
    } else if (c == 'R') {
        RA4M1_Hw::resetHardwareEncoder();
        engine.reset(0);
        contacts = 0;
        contact_refractory = 0;
    } else if (c == 'P' && selftest_ok && polarity_ticks == 0) {
        RA4M1_Hw::resetHardwareEncoder();
        polarity_ticks = 1;
    }
}

static void controlIsr() {
    const uint32_t t_entry = RA4M1_Hw::pacerCount();
    R_BSP_IrqStatusClear(control_irq.irq);        // IELSR.IR = 0

    RA4M1_Hw::kickWatchdog();

    const uint16_t grip_adc = RA4M1_Hw::readGripResult();
    int32_t counts = RA4M1_Hw::readLatchedEncoder();
    const bool edge = RA4M1_Hw::takeContactEdge();

    const char req = cmd_request;
    if (req) {
        cmd_request = 0;
        handleRequest(req);
        counts = RA4M1_Hw::readHardwareEncoder();
    }

    if (contact_refractory) contact_refractory--;
    if (edge && contact_refractory == 0) {
        contacts++;
        contact_refractory = CONTACT_REFRACTORY_TICKS;
    }

    MotorCommand cmd = engine.step(grip_adc, counts);
    const bool poeg = RA4M1_Hw::poegTripped();
    const bool armed = selftest_ok && polarity_ok && !poeg;

    if (polarity_ticks) {
        RA4M1_Hw::bridgeStandby(!poeg);
        if (polarity_ticks <= POLARITY_DRIVE_TICKS) driveBridge(0.20f); else coastBridge();
        if (++polarity_ticks > POLARITY_TOTAL_TICKS) {
            coastBridge();
            polarity_result = RA4M1_Hw::readHardwareEncoder();
            polarity_done = true;
            polarity_ticks = 0;
            RA4M1_Hw::resetHardwareEncoder();
            engine.reset(0);
        }
    } else {
        RA4M1_Hw::bridgeStandby(armed);
        if (!armed || cmd.coast) coastBridge(); else driveBridge(cmd.duty);
    }

    const uint32_t s = snap_seq;
    snap_seq = s + 1;
    snap.tick = isr_ticks;
    snap.grip_adc = grip_adc;
    snap.counts = counts;
    snap.mode = engine.mode();
    snap.vel = engine.velocity();
    snap.damping = engine.damping();
    snap.difficulty = engine.difficulty();
    snap.tremor = engine.tremorRms();
    snap.duty = cmd.coast ? 0.0f : cmd.duty;
    snap.energy = engine.observerEnergy();
    snap.contacts = contacts;
    snap.pc_activations = engine.pcActivations();
    snap.fault = engine.fault();
    snap_seq = s + 2;
    isr_ticks = isr_ticks + 1;

    if (t_entry > max_latency_counts) max_latency_counts = t_entry;
    const uint32_t t_exit = RA4M1_Hw::pacerCount();
    if (t_exit < t_entry) {                       // crossed a pacer boundary: a tick was lost
        overruns = overruns + 1;
        RA4M1_Hw::poegTrip();                     // hardware latch: PWM forced LOW until reset
    } else if (t_exit - t_entry > max_isr_counts) {
        max_isr_counts = t_exit - t_entry;
    }
}

static bool readSnapshot(Snapshot& out) {
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint32_t s0 = snap_seq;
        if (s0 & 1U) continue;
        out = snap;
        __DMB();
        if (snap_seq == s0) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------------------------


#if (SG_TELEMETRY_SINK & 2)
static char udp_buf[TELEMETRY_ROW_MAX * TELEMETRY_UDP_BATCH];
static uint16_t udp_len = 0;
static uint8_t udp_rows = 0;
#endif

static void emit(const RowWriter& w) {
#if (SG_TELEMETRY_SINK & 1)
    Serial.write((const uint8_t*)w.buf, w.n);     // blocking is acceptable: control runs in the ISR
#endif
#if (SG_TELEMETRY_SINK & 2)
    if (!wifi_ok) return;
    memcpy(udp_buf + udp_len, w.buf, w.n);
    udp_len += w.n;
    if (++udp_rows >= TELEMETRY_UDP_BATCH) {
        udp.beginPacket(SECRET_UDP_HOST, TELEMETRY_UDP_PORT);
        udp.write((const uint8_t*)udp_buf, udp_len);
        udp.endPacket();
        udp_len = 0;
        udp_rows = 0;
    }
#endif
}

// ---------------------------------------------------------------------------------------------
// 0.1 Hz breathing pacer on the 12x8 matrix: 4 s inhale (ring grows), 6 s exhale (ring shrinks).
// ---------------------------------------------------------------------------------------------
static void updatePacer(float difficulty) {
    static unsigned long last_ms = 0;
    static int last_radius = -2;
    const unsigned long now = millis();
    if (now - last_ms < 100) return;
    last_ms = now;

    int radius = -1;
    if (difficulty >= 0.20f) {
        const float phase = (float)(now % 10000UL) / 1000.0f;
        const float level = (phase < 4.0f) ? (phase / 4.0f) : (1.0f - (phase - 4.0f) / 6.0f);
        radius = (int)lroundf(level * 4.0f);
    }
    if (radius == last_radius) return;
    last_radius = radius;

    uint8_t frame[8][12] = {{0}};
    const int r = radius < 0 ? 0 : radius;
    for (int y = 3 - r; y <= 4 + r; ++y)
        for (int x = 5 - r; x <= 6 + r; ++x)
            if (y >= 0 && y < 8 && x >= 0 && x < 12) frame[y][x] = 1;
    matrix.renderBitmap(frame, 8, 12);
}

static void handleCommands() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == 'A' || c == 'B' || c == 'C' || c == 'R') {
            cmd_request = c;
        } else if (c == 'P') {
            if (!selftest_ok) Serial.println(F("#POLARITY,SKIPPED,selftest_failed"));
            else cmd_request = c;
        } else if (c == 'S') {
            selftest_ok = RA4M1_Hw::runRegisterSelfTest(Serial) && selftest_ok;
        }
    }
    if (polarity_done) {
        polarity_done = false;
        const int32_t moved = polarity_result;
        Serial.print(F("#POLARITY,counts="));
        Serial.print(moved);
        if (moved >= 20) {
            polarity_ok = true;
            Serial.println(F(",PASS"));
        } else if (moved <= -20) {
            polarity_ok = false;
            Serial.println(F(",INVERTED,swap_motor_leads_or_set_MOTOR_POLARITY_INVERT"));
        } else {
            polarity_ok = false;
            Serial.println(F(",NO_MOTION,check_VM_STBY_wiring"));
        }
    }
}

void setup() {
    pinMode(PIN_TB_STBY, OUTPUT);
    pinMode(PIN_TB_AIN1, OUTPUT);
    pinMode(PIN_TB_AIN2, OUTPUT);
    RA4M1_Hw::bridgeStandby(false);               // driver disabled until armed
    RA4M1_Hw::bridgeDirection(0);

    Serial.begin(TELEMETRY_BAUD);
    const unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}
    Serial.println(F("# SymbioGrip v4 hardware pipeline (UNO R4 WiFi) boot"));

    // Timers are reserved before matrix.begin() so FspTimer hands the matrix a free GPT (GPT4).
    RA4M1_Hw::initHardwareQuadratureEncoder();
    RA4M1_Hw::initPacer();
    RA4M1_Hw::initPwmAndContact();
    RA4M1_Hw::initPoeg();
    matrix.begin();

    const bool opamp_ok = RA4M1_Hw::initInternalOpamp();
    RA4M1_Hw::initAdcBase();
    float avcc = 0.0f;
    const bool analog_ok = RA4M1_Hw::runAnalogSelfTest(Serial, avcc);
    RA4M1_Hw::initElcLinks();
    RA4M1_Hw::armAdcPipeline();

    Serial.print(F("#SELFTEST,OPAMP.begin,")); Serial.println(opamp_ok ? F("PASS") : F("FAIL"));
    selftest_ok = RA4M1_Hw::runRegisterSelfTest(Serial) && opamp_ok && analog_ok;
    const bool irq_ok = IRQManager::getInstance().addGenericInterrupt(control_irq, controlIsr);
    Serial.print(F("#SELFTEST,IRQ.ADC0_SCAN_END,")); Serial.println(irq_ok ? F("PASS") : F("FAIL"));
    selftest_ok = selftest_ok && irq_ok;
    Serial.println(selftest_ok ? F("#SELFTEST,RESULT,PASS") : F("#SELFTEST,RESULT,FAIL_motor_disabled"));

#if (SG_TELEMETRY_SINK & 2)
    if (sizeof(SECRET_SSID) > 1) {
        wifi_ok = (WiFi.begin(SECRET_SSID, SECRET_PASS) == WL_CONNECTED) && udp.begin(TELEMETRY_UDP_PORT);
    }
    Serial.println(wifi_ok ? F("#WIFI,UDP,PASS") : F("#WIFI,UDP,FAIL_or_no_secrets"));
#endif

    engine.setMode('C', 0);
    RA4M1_Hw::resetHardwareEncoder();
    if (irq_ok) {
        RA4M1_Hw::startPacerAndPwm();
        RA4M1_Hw::startWatchdog();                // only the control ISR refreshes it from now on
    }
    Serial.println(F("# Send 'P' with the carriage free to verify actuator polarity and arm the motor."));
    Serial.println(F("t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts"));
}

void loop() {
    static uint32_t last_row_tick = 0, last_stat_tick = 0, drops = 0;
    handleCommands();

    Snapshot s;
    if (!readSnapshot(s)) return;
    updatePacer(s.difficulty);

    if (s.tick - last_row_tick >= TELEMETRY_DIV) {
        const uint32_t missed = (s.tick - last_row_tick) / TELEMETRY_DIV - 1;
        if (last_row_tick != 0) drops += missed;
        last_row_tick = s.tick - (s.tick % TELEMETRY_DIV);
        RowWriter w;
        w.u32(s.tick * 2U); w.sep();              // t_ms from the hardware tick count (2 ms per tick)
        w.ch(s.mode); w.sep();
        w.u32(s.grip_adc); w.sep();
        w.i32(s.counts); w.sep();
        w.fixed(s.vel, 4); w.sep();
        w.fixed(s.damping, 2); w.sep();
        w.fixed(s.difficulty, 3); w.sep();
        w.fixed(s.tremor, 3); w.sep();
        w.fixed(s.duty, 3); w.sep();
        w.fixed(s.energy * 1000.0f, 2); w.sep();
        w.i32(s.contacts);
        w.ch('\n');
        emit(w);
    }

    if (s.tick - last_stat_tick >= 500) {
        last_stat_tick = s.tick;
        RowWriter w;
        w.str("#STAT,"); w.u32(s.tick * 2U); w.sep();
        w.u32(overruns); w.sep(); w.u32(drops); w.sep();
        w.u32(max_isr_counts / (HW_PCLKD_HZ / 1000000UL)); w.sep();
        w.u32(s.pc_activations); w.sep(); w.u32(s.fault); w.sep();
        w.u32((selftest_ok && polarity_ok && !RA4M1_Hw::poegTripped()) ? 1 : 0); w.sep();
        w.u32(max_latency_counts / (HW_PCLKD_HZ / 1000000UL)); w.sep();
        w.u32(RA4M1_Hw::poegTripped() ? 1 : 0);
        w.ch('\n');
        emit(w);
    }
}
