/*
 * OllO frame protocol - shared by the ESP32 (sender) and the RP2040 (receiver).
 *
 * The ESP32 renders the whole UI into a 320x240 8-bit framebuffer whose values are
 * indices into this fixed palette, and streams it to the RP2040:
 *
 *   0         = black
 *   1..216    = 6x6x6 colour cube   (index = 1 + r*36 + g*6 + b, each 0..5)
 *   217..254  = 38-step gray ramp   (keeps text anti-aliasing smooth)
 *   255       = white
 *
 * Wire format (ESP32 -> RP2040):
 *   A5 5A 'F'
 *   240 x row:   [enc]            enc 0 = RAW : 320 bytes follow
 *                                 enc 1 = RLE : [len u16 LE] then len bytes of (count, value) pairs
 *   [checksum]   sum of all 76800 decoded pixel bytes, modulo 256
 */
#ifndef OLLO_FRAME_PROTOCOL_H
#define OLLO_FRAME_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define OLLO_FRAME_W   320
#define OLLO_FRAME_H   240
#define OLLO_MAGIC0    0xA5
#define OLLO_MAGIC1    0x5A
#define OLLO_CMD_FRAME 'F'
#define OLLO_ROW_RAW   0
#define OLLO_ROW_RLE   1

/* Gray level 0..255 -> palette index (black / gray ramp / white). */
static inline uint8_t olloGrayToPalette(uint8_t g) {
    if (g <= 3)   return 0;
    if (g >= 251) return 255;
    int idx = 216 + ((int)g * 39 + 127) / 255;
    if (idx < 217) idx = 217;
    if (idx > 254) idx = 254;
    return (uint8_t)idx;
}

/* RGB565 pixel -> palette index. Near-neutral pixels use the gray ramp. */
static inline uint8_t olloRgb565ToPalette(uint16_t p) {
    uint8_t r = (p >> 11) & 0x1F;
    uint8_t g = (p >> 5) & 0x3F;
    uint8_t b = p & 0x1F;
    r = (uint8_t)((r << 3) | (r >> 2));
    g = (uint8_t)((g << 2) | (g >> 4));
    b = (uint8_t)((b << 3) | (b >> 2));

    const uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    const uint8_t mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    if ((int)mx - (int)mn <= 10)
        return olloGrayToPalette((uint8_t)(((int)r + g + b) / 3));

    const int rq = (r * 5 + 127) / 255;
    const int gq = (g * 5 + 127) / 255;
    const int bq = (b * 5 + 127) / 255;
    if (rq == 0 && gq == 0 && bq == 0) return 0;
    return (uint8_t)(1 + rq * 36 + gq * 6 + bq);
}

/* RLE-encode one row. Returns the encoded length, or 0 if it is not smaller than raw. */
static inline size_t olloRleEncodeRow(const uint8_t* row, uint8_t* out) {
    size_t n = 0;
    int x = 0;
    while (x < OLLO_FRAME_W) {
        const uint8_t v = row[x];
        int c = 1;
        while (x + c < OLLO_FRAME_W && row[x + c] == v && c < 255) c++;
        if (n + 2 >= OLLO_FRAME_W) return 0;
        out[n++] = (uint8_t)c;
        out[n++] = v;
        x += c;
    }
    return n;
}

/* Decode one RLE row into exactly OLLO_FRAME_W pixels. */
static inline bool olloRleDecodeRow(const uint8_t* in, size_t len, uint8_t* row) {
    if (len == 0 || (len & 1)) return false;
    int x = 0;
    for (size_t i = 0; i < len; i += 2) {
        const int c = in[i];
        if (c == 0 || x + c > OLLO_FRAME_W) return false;
        memset(row + x, in[i + 1], (size_t)c);
        x += c;
    }
    return x == OLLO_FRAME_W;
}

#endif /* OLLO_FRAME_PROTOCOL_H */
