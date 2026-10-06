/*
 * OllO - RP2040 display driver (1-bit mono, 640x480, double buffered)
 *
 * Arduino-Pico + PicoDVI (DVIGFX1).  This chip does NO rendering: the ESP32 draws the
 * whole UI (LVGL) and streams finished 640x480 black/white frames over UART.  Each
 * frame is received into the back buffer and flipped to the screen only when complete
 * and checksum-verified, so a bad frame is never shown.
 *
 * Protocol: include/ollo_frame.h
 *
 * Wiring:  ESP32 GPIO4 (TX) -> RP2040 GP13 (RX)
 *          ESP32 GPIO5 (RX) <- RP2040 GP12 (TX)
 */

#include <Arduino.h>
#include <PicoDVI.h>
#include "ollo_frame.h"

const struct dvi_serialiser_cfg my_dvi_cfg = {
    .pio = pio0,
    .sm_tmds = {0, 1, 2},
    .pins_tmds = {2, 4, 6},
    .pins_clk = 8,
    .invert_diffpairs = true
};

DVIGFX1 display(DVI_RES_640x480p60, true, my_dvi_cfg);   // true = double buffered

#define UART_TX_PIN 12
#define UART_RX_PIN 13
#define UART_BAUD 2000000   // must match the ESP32 side

static uint8_t rleBuf[OLLO_ROW_BYTES + 8];
static uint32_t framesOk = 0;
static uint32_t framesBad = 0;
static const char* lastError = "";

static bool readExact(uint8_t* dst, size_t n) {
    return Serial1.readBytes(dst, n) == n;   // honours Serial1.setTimeout()
}

/* Reads one frame (everything after the 3 header bytes) into the back buffer. */
static bool receiveFrame() {
    uint8_t* fb = display.getBuffer();
    uint8_t sum = 0;

    for (int y = 0; y < OLLO_FRAME_H; y++) {
        uint8_t* row = fb + (size_t)y * OLLO_ROW_BYTES;
        uint8_t enc;
        if (!readExact(&enc, 1)) { lastError = "timeout (row type)"; return false; }

        if (enc == OLLO_ROW_RAW) {
            if (!readExact(row, OLLO_ROW_BYTES)) { lastError = "timeout (raw row)"; return false; }
        } else if (enc == OLLO_ROW_RLE) {
            uint8_t lenBytes[2];
            if (!readExact(lenBytes, 2)) { lastError = "timeout (rle length)"; return false; }
            const size_t len = (size_t)lenBytes[0] | ((size_t)lenBytes[1] << 8);
            if (len == 0 || len > sizeof(rleBuf)) { lastError = "bad rle length"; return false; }
            if (!readExact(rleBuf, len)) { lastError = "timeout (rle data)"; return false; }
            if (!olloRleDecodeRow(rleBuf, len, row)) { lastError = "rle decode failed"; return false; }
        } else {
            lastError = "unknown row type";
            return false;
        }

        for (int x = 0; x < OLLO_ROW_BYTES; x++) sum += row[x];
    }

    uint8_t expected;
    if (!readExact(&expected, 1)) { lastError = "timeout (checksum)"; return false; }
    if (expected != sum) { lastError = "checksum mismatch"; return false; }
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("A: Ollo RP2040 display driver (mono 640x480 frame receiver)");

    Serial.println("B: before display.begin");
    if (!display.begin()) {   // also sets the ~252 MHz system clock
        Serial.println("DVI begin FAILED (out of RAM?)");
        while (true) delay(1000);
    }
    Serial.println("C: after display.begin");

    // Splash until the ESP32 sends the first frame (white screen, black text)
    display.fillScreen(OLLO_BIT_WHITE);
    display.setTextColor(OLLO_BIT_WHITE ? 0 : 1);
    display.setTextSize(8);
    display.setCursor(OLLO_FRAME_W / 2 - 96, OLLO_FRAME_H / 2 - 28);
    display.print("OllO");
    display.swap(false);

    // UART comes AFTER display.begin(): that call changes the system clock, and a baud
    // rate set up before it would come out wrong.
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.setFIFOSize(8192);
    Serial1.setTimeout(500);
    Serial1.begin(UART_BAUD);

    Serial1.print("READY\n");   // ask the ESP32 to (re)send the current screen
    Serial.println("Waiting for frames from the ESP32");
}

void loop() {
    static uint8_t state = 0;
    static uint32_t lastReport = 0;
    static uint32_t lastResend = 0;
    static uint32_t rxBytes = 0;

    while (Serial1.available()) {
        const uint8_t c = (uint8_t)Serial1.read();
        rxBytes++;

        if (state == 0) {
            state = (c == OLLO_MAGIC0) ? 1 : 0;
        } else if (state == 1) {
            state = (c == OLLO_MAGIC1) ? 2 : (c == OLLO_MAGIC0 ? 1 : 0);
        } else {
            state = 0;
            if (c == OLLO_CMD_FRAME) {
                if (receiveFrame()) {
                    display.swap(false);   // show the finished frame
                    framesOk++;
                } else {
                    framesBad++;
                    Serial.print("Frame dropped: ");
                    Serial.println(lastError);
                    // The bad frame was never shown; ask for the screen again (rate limited).
                    if (millis() - lastResend > 500) {
                        lastResend = millis();
                        while (Serial1.available()) Serial1.read();
                        Serial1.print("READY\n");
                    }
                }
            }
        }
    }

    if (millis() - lastReport > 5000) {
        lastReport = millis();
        Serial.print("frames ok=");
        Serial.print(framesOk);
        Serial.print(" dropped=");
        Serial.print(framesBad);
        Serial.print(" rx bytes=");
        Serial.println(rxBytes);
    }
}
