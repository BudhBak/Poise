// SymbioGrip ESP32-S3 telemetry bridge (OPTIONAL, NOT REQUIRED, NOT COMPILED OR TESTED).
//
// The default WiFi path needs no ESP32 changes: build the RA4M1 sketch with SG_TELEMETRY_SINK = 2 or 3
// and the stock ESP32-S3 firmware sends UDP through the WiFiS3 AT link (RA4M1 Serial2 = SCI1, P501/P502).
//
// This sketch REPLACES the stock ESP32-S3 firmware. Consequences:
//   * The stock firmware also bridges USB-C to the RA4M1 (upload + Serial). After replacing it, the RA4M1
//     can no longer be programmed over USB until the stock firmware is reflashed (Arduino "UNO R4 WiFi
//     firmware updater" / espflash). Do this only if you can reflash the ESP32-S3.
//   * WiFiS3 on the RA4M1 stops working (no AT server). Build the RA4M1 sketch with SG_TELEMETRY_SINK = 1.
//
// Function: read the RA4M1 telemetry CSV lines arriving on the ESP32-S3 UART that is wired to RA4M1
// Serial (SCI9, P109/P110), batch TELEMETRY_UDP_BATCH lines, send them as one UDP datagram.
//
// The ESP32-S3 GPIO numbers of that link are NOT verifiable from the files on this machine (no ESP32 core,
// no board schematic). They must be read from the official UNO R4 WiFi schematic and entered below; the
// static_assert stops the build until that is done.

#include <WiFi.h>
#include <WiFiUdp.h>

#if __has_include("secrets.h")
#include "secrets.h"            // SECRET_SSID, SECRET_PASS, SECRET_UDP_HOST
#endif
#ifndef SECRET_SSID
#define SECRET_SSID ""
#define SECRET_PASS ""
#endif
#ifndef SECRET_UDP_HOST
#define SECRET_UDP_HOST "255.255.255.255"
#endif

static constexpr int LINK_RX_GPIO = -1;          // ESP32-S3 GPIO receiving RA4M1 P109 (TXD9): FROM SCHEMATIC
static constexpr int LINK_TX_GPIO = -1;          // ESP32-S3 GPIO driving RA4M1 P110 (RXD9): FROM SCHEMATIC
static_assert(LINK_RX_GPIO >= 0 && LINK_TX_GPIO >= 0,
              "Set LINK_RX_GPIO / LINK_TX_GPIO from the UNO R4 WiFi schematic before building");

static constexpr uint32_t LINK_BAUD = 115200;    // = TELEMETRY_BAUD in firmware/src/config.h
static constexpr uint16_t UDP_PORT = 5005;       // = TELEMETRY_UDP_PORT
static constexpr int BATCH = 5;                  // = TELEMETRY_UDP_BATCH
static constexpr size_t LINE_MAX = 96;           // = TELEMETRY_ROW_MAX

static WiFiUDP udp;
static char batch_buf[LINE_MAX * BATCH];
static size_t batch_len = 0;
static int batch_rows = 0;
static char line[LINE_MAX];
static size_t line_len = 0;

static void flushBatch() {
    if (batch_len == 0 || WiFi.status() != WL_CONNECTED) return;
    udp.beginPacket(SECRET_UDP_HOST, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(batch_buf), batch_len);
    udp.endPacket();
    batch_len = 0;
    batch_rows = 0;
}

void setup() {
    Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_GPIO, LINK_TX_GPIO);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SECRET_SSID, SECRET_PASS);
}

void loop() {
    while (Serial1.available() > 0) {
        const char c = static_cast<char>(Serial1.read());
        if (line_len < LINE_MAX - 1) line[line_len++] = c;
        if (c != '\n') continue;
        if (batch_len + line_len <= sizeof(batch_buf)) {
            memcpy(batch_buf + batch_len, line, line_len);
            batch_len += line_len;
        }
        line_len = 0;
        if (line[0] == '#' || ++batch_rows >= BATCH) flushBatch();   // status lines are sent immediately
    }
}
