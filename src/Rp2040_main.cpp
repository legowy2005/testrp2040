/*
 * OllO - RP2040 display driver
 *
 * Arduino-Pico + PicoDVI.  This chip does NO rendering: the ESP32 draws the whole
 * UI (LVGL) and streams finished 320x240 frames over UART.  The RP2040 copies each
 * frame into the back buffer and flips it to the screen when it is complete.
 *
 * Protocol and palette: include/ollo_frame.h
 *
 * Wiring:  ESP32 GPIO4 (TX) -> RP2040 GP13 (RX)
 *          ESP32 GPIO5 (RX) <- RP2040 GP12 (TX)
 */

#include <Arduino.h>
#include <PicoDVI.h>
#include "ollo_frame.h"

// ============================================================
// DVI (320x240, 8-bit palette, double buffered)
// ============================================================

const struct dvi_serialiser_cfg my_dvi_cfg = {
    .pio = pio0,
    .sm_tmds = {0, 1, 2},
    .pins_tmds = {2, 4, 6},
    .pins_clk = 8,
    .invert_diffpairs = true
};

DVIGFX8 display(DVI_RES_320x240p60, true, my_dvi_cfg);

// ============================================================
// UART
// ============================================================

#define UART_TX_PIN 12
#define UART_RX_PIN 13
#define UART_BAUD 1000000   // must match the ESP32 side

// ============================================================
// Palette (see ollo_frame.h)
// ============================================================

static void initColorPalette() {
    display.setColor(0, 0, 0, 0);

    uint8_t index = 1;
    for (int r = 0; r < 6; r++)
        for (int g = 0; g < 6; g++)
            for (int b = 0; b < 6; b++)
                display.setColor(index++, (r * 255) / 5, (g * 255) / 5, (b * 255) / 5);

    for (; index < 255; index++) {
        const uint8_t gray = (uint8_t)(((index - 216) * 255) / 39);
        display.setColor(index, gray, gray, gray);
    }

    display.setColor(255, 255, 255, 255);
}

// ============================================================
// Frame receiver
// ============================================================

static uint8_t rleBuf[OLLO_FRAME_W + 8];
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
        uint8_t* row = fb + (size_t)y * OLLO_FRAME_W;
        uint8_t enc;
        if (!readExact(&enc, 1)) { lastError = "timeout (row type)"; return false; }

        if (enc == OLLO_ROW_RAW) {
            if (!readExact(row, OLLO_FRAME_W)) { lastError = "timeout (raw row)"; return false; }
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

        for (int x = 0; x < OLLO_FRAME_W; x++) sum += row[x];
    }

    uint8_t expected;
    if (!readExact(&expected, 1)) { lastError = "timeout (checksum)"; return false; }
    if (expected != sum) { lastError = "checksum mismatch"; return false; }
    return true;
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Ollo RP2040 display driver (frame receiver)");

    if (!display.begin()) {   // also sets the ~252 MHz system clock
        Serial.println("DVI begin FAILED");
        while (true) delay(1000);
    }

    initColorPalette();
    display.swap(false, true);   // front + back buffer now share the palette

    // Splash until the ESP32 sends the first frame
    display.fillScreen(255);
    display.setTextColor(0);
    display.setTextSize(3);
    display.setCursor(104, 100);
    display.print("OllO");
    display.swap(false, false);

    // UART comes AFTER display.begin(): that call changes the system clock, and a baud
    // rate set up before it would come out wrong.
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.setFIFOSize(4096);
    Serial1.setTimeout(500);
    Serial1.begin(UART_BAUD);

    Serial1.print("READY\n");   // ask the ESP32 to (re)send the current screen
    Serial.println("Waiting for frames from the ESP32");
}

void loop() {
    static uint8_t state = 0;
    static uint32_t lastReport = 0;

    while (Serial1.available()) {
        const uint8_t c = (uint8_t)Serial1.read();

        if (state == 0) {
            state = (c == OLLO_MAGIC0) ? 1 : 0;
        } else if (state == 1) {
            state = (c == OLLO_MAGIC1) ? 2 : (c == OLLO_MAGIC0 ? 1 : 0);
        } else {
            state = 0;
            if (c == OLLO_CMD_FRAME) {
                if (receiveFrame()) {
                    display.swap(false, false);   // show the finished frame
                    framesOk++;
                } else {
                    framesBad++;
                    Serial.print("Frame dropped: ");
                    Serial.println(lastError);
                }
            }
        }
    }

    if (millis() - lastReport > 5000) {
        lastReport = millis();
        Serial.print("frames ok=");
        Serial.print(framesOk);
        Serial.print(" dropped=");
        Serial.println(framesBad);
    }
}
