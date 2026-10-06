/*
 * OllO frame protocol - shared by the ESP32 (sender) and the RP2040 (receiver).
 *
 * 2-bit grayscale (4 levels) 640x480 frames, 4 pixels per byte.
 * Pixel x of a row lives in byte (x >> 2), at bit offset (x & 3) * 2, i.e. the LEFTMOST pixel
 * is in the LEAST significant bits.  That is exactly the layout PicoDVI's tmds_encode_2bpp()
 * reads, so the RP2040 can scan the received buffer out without reshuffling it.
 * Level 0 = black, 1 = dark gray, 2 = light gray, 3 = white.
 *
 * Wire format (ESP32 -> RP2040):
 *   A5 5A 'G'
 *   OLLO_FRAME_H x row:  [enc]   enc 0 = RAW : OLLO_ROW_BYTES bytes follow
 *                                enc 1 = RLE : [len u16 LE] then len bytes of (count, value) pairs
 *   [checksum]   sum of all decoded bytes (H * ROW_BYTES of them), modulo 256
 *
 * (The old 1-bit protocol used 'F'; the different command byte means a half-updated pair of
 *  boards simply shows nothing instead of garbage.)
 */
#ifndef OLLO_FRAME_PROTOCOL_H
#define OLLO_FRAME_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define OLLO_FRAME_W     640
#define OLLO_FRAME_H     480
#define OLLO_ROW_BYTES   (OLLO_FRAME_W / 4)
#define OLLO_FRAME_BYTES ((size_t)OLLO_ROW_BYTES * OLLO_FRAME_H)

#define OLLO_LEVEL_BLACK 0
#define OLLO_LEVEL_WHITE 3
#define OLLO_FILL_WHITE  0xFF      /* every pixel = level 3 */

#define OLLO_MAGIC0      0xA5
#define OLLO_MAGIC1      0x5A
#define OLLO_CMD_FRAME   'G'
#define OLLO_ROW_RAW     0
#define OLLO_ROW_RLE     1

/* RGB565 pixel -> 8-bit luma. */
static inline uint8_t olloRgb565ToGray(uint16_t p) {
    uint8_t r = (p >> 11) & 0x1F;
    uint8_t g = (p >> 5) & 0x3F;
    uint8_t b = p & 0x1F;
    r = (uint8_t)((r << 3) | (r >> 2));
    g = (uint8_t)((g << 2) | (g >> 4));
    b = (uint8_t)((b << 3) | (b >> 2));
    return (uint8_t)((r * 77 + g * 151 + b * 28) >> 8);
}

/* 8-bit luma -> 2-bit level (0..3), rounded to the nearest level. */
static inline uint8_t olloGrayToLevel(uint8_t gray) {
    return (uint8_t)(((unsigned)gray * 3u + 127u) / 255u);
}

/* Set one pixel (level 0..3) in a packed 2-bit framebuffer (OLLO_ROW_BYTES per row). */
static inline void olloPutLevel(uint8_t* fb, int x, int y, uint8_t level) {
    uint8_t* p = fb + (size_t)y * OLLO_ROW_BYTES + (x >> 2);
    const uint8_t shift = (uint8_t)((x & 3) * 2);
    *p = (uint8_t)((*p & ~(0x03 << shift)) | ((level & 0x03) << shift));
}

/* Black/white convenience wrapper (used for the 1-bit card images). */
static inline void olloPutPixel(uint8_t* fb, int x, int y, bool white) {
    olloPutLevel(fb, x, y, white ? OLLO_LEVEL_WHITE : OLLO_LEVEL_BLACK);
}

/* RLE-encode one packed row. Returns the encoded length, or 0 if it is not smaller than raw. */
static inline size_t olloRleEncodeRow(const uint8_t* row, uint8_t* out) {
    size_t n = 0;
    int x = 0;
    while (x < OLLO_ROW_BYTES) {
        const uint8_t v = row[x];
        int c = 1;
        while (x + c < OLLO_ROW_BYTES && row[x + c] == v && c < 255) c++;
        if (n + 2 >= OLLO_ROW_BYTES) return 0;
        out[n++] = (uint8_t)c;
        out[n++] = v;
        x += c;
    }
    return n;
}

/* Decode one RLE row into exactly OLLO_ROW_BYTES bytes. */
static inline bool olloRleDecodeRow(const uint8_t* in, size_t len, uint8_t* row) {
    if (len == 0 || (len & 1)) return false;
    int x = 0;
    for (size_t i = 0; i < len; i += 2) {
        const int c = in[i];
        if (c == 0 || x + c > OLLO_ROW_BYTES) return false;
        memset(row + x, in[i + 1], (size_t)c);
        x += c;
    }
    return x == OLLO_ROW_BYTES;
}

#endif /* OLLO_FRAME_PROTOCOL_H */
