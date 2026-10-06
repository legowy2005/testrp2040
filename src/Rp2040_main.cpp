/*
 * OllO - RP2040 display driver (2-bit gray, 640x480, double buffered)
 *
 * Arduino-Pico + PicoDVI (custom DVIGFX2 class below, built on the library's tmds_encode_2bpp).
 * This chip does NO rendering: the ESP32 draws the whole UI (LVGL) and streams finished
 * 640x480 4-level gray frames over UART.  Each
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
#include "libdvi/tmds_encode.h"   // tmds_encode_2bpp()
#include "ollo_frame.h"

const struct dvi_serialiser_cfg my_dvi_cfg = {
    .pio = pio0,
    .sm_tmds = {0, 1, 2},
    .pins_tmds = {2, 4, 6},
    .pins_clk = 8,
    .invert_diffpairs = true
};

/*
 * Minimal 2-bit grayscale framebuffer for PicoDVI (the library only ships 1/8/16-bit classes,
 * but libdvi already contains an assembly 2bpp->TMDS encoder meant for monochrome TMDS).
 * Same structure as the library's DVIGFX1: core 1 runs the DVI engine, _mainloop() (started
 * on core 1 by the library's setup1()) encodes one scanline at a time from the front buffer.
 *
 * Pixel layout: 4 pixels per byte, leftmost pixel in the lowest 2 bits (see ollo_frame.h).
 * Level 0 = black ... 3 = white.  Two buffers: 2 x 76.8 KB.
 */
class DVIGFX2 : public PicoDVI {
public:
    explicit DVIGFX2(const struct dvi_serialiser_cfg &c)
        : PicoDVI(dvi_timing_640x480p_60hz, c, VREG_VOLTAGE_1_20) {
        dvi_vertical_repeat = 1;
        dvi_monochrome_tmds = true;
    }

    bool begin();
    void _mainloop();                         // runs on core 1, never returns

    uint8_t* getBuffer() { return (uint8_t*)bufs[1 - front]; }   // the BACK buffer (draw here)
    void swap(bool = false) {                 // show the back buffer at the next frame end
        for (swapWait = true; swapWait;) {}
    }

private:
    static constexpr int W = OLLO_FRAME_W;
    static constexpr int H = OLLO_FRAME_H;
    alignas(4) static uint32_t bufs[2][OLLO_FRAME_BYTES / 4];
    volatile uint8_t front = 0;               // buffer currently scanned out
    volatile bool swapWait = false;
};

alignas(4) uint32_t DVIGFX2::bufs[2][OLLO_FRAME_BYTES / 4];
static DVIGFX2* dvi2ptr = nullptr;

static void mainloop2(struct dvi_inst*) { dvi2ptr->_mainloop(); }

void __not_in_flash_func(DVIGFX2::_mainloop)() {
    for (;;) {
        const uint8_t* fb = (const uint8_t*)bufs[front];
        for (int y = 0; y < H; y++) {
            uint32_t* tmdsbuf;
            queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);
            tmds_encode_2bpp((const uint32_t*)(fb + (size_t)y * OLLO_ROW_BYTES), tmdsbuf, W);
            queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
        }
        if (swapWait) {          // swap between frames, never mid-frame
            front = 1 - front;
            swapWait = false;
        }
    }
}

bool DVIGFX2::begin() {
    memset(bufs, OLLO_FILL_WHITE, sizeof(bufs));
    dvi2ptr = this;
    mainloop = mainloop2;
    PicoDVI::begin();            // sets voltage + ~252 MHz system clock, inits DVI
    wait_begin = false;          // set core 1 in motion
    return true;
}

DVIGFX2 display(my_dvi_cfg);

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
    Serial.println("A: Ollo RP2040 display driver (2-bit gray 640x480 frame receiver)");

    Serial.println("B: before display.begin");
    if (!display.begin()) {   // also sets the ~252 MHz system clock
        Serial.println("DVI begin FAILED (out of RAM?)");
        while (true) delay(1000);
    }
    Serial.println("C: after display.begin");

    // Test pattern until the ESP32 sends the first frame: white screen with four gray bars
    // (levels 0,1,2,3 left to right) so you can see at a glance that all 4 levels work.
    {
        uint8_t* fb = display.getBuffer();
        memset(fb, OLLO_FILL_WHITE, OLLO_FRAME_BYTES);
        for (int y = 200; y < 280; y++)
            for (int x = 0; x < OLLO_FRAME_W; x++)
                olloPutLevel(fb, x, y, (uint8_t)(x / (OLLO_FRAME_W / 4)));
    }
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
