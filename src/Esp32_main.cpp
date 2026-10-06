/*
 * OllO - ESP32-S3 Super Mini
 * BLE deck sync + LittleFS storage + UART display bridge + touch pads
 *
 * Protocol v2 additions:
 *   - CARD packets can carry a folder name.
 *   - Images carry a format byte: 0 = 1-bit, 1 = RGB565 color.
 *   - RGB565 images are streamed to the RP2040; the ESP32 never needs
 *     a full-image RAM buffer.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lvgl.h>
#include "ollo_frame.h"

// ============================================================
// BLE UUIDs
// ============================================================

static const char* SERVICE_UUID =
    "6f6c6c6f-0001-4000-8000-00805f9b34fb";

static const char* WRITE_UUID =
    "6f6c6c6f-0002-4000-8000-00805f9b34fb";

static const char* NOTIFY_UUID =
    "6f6c6c6f-0003-4000-8000-00805f9b34fb";

// ============================================================
// UART
// ============================================================

#define UART_TX 4
#define UART_RX 5
#define UART_BAUD 2000000   // must match the RP2040 side (higher = faster screen updates)

HardwareSerial RPSerial(1);

// ============================================================
// Touch pins
// ============================================================

#define TOUCH_BACK_PIN   6
#define TOUCH_FRONT_PIN  7

// ============================================================
// Limits / image formats
// ============================================================

#define MAX_TEXT 100
#define MAX_FOLDER_NAME 20

#define IMAGE_FORMAT_MONO   0
#define IMAGE_FORMAT_RGB565 1

#define MAX_MONO_IMAGE_BYTES 38400UL       // 640x480 @ 1 bit
#define MAX_COLOR_IMAGE_BYTES 153600UL     // 320x240 @ RGB565
#define MAX_IMAGE_BYTES MAX_COLOR_IMAGE_BYTES

#define MAX_IMG_W 640
#define MAX_IMG_H 480
#define MAX_COLOR_IMG_W 320
#define MAX_COLOR_IMG_H 240
#define MAX_CARDS 65535

// ============================================================
// BLE
// ============================================================

NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* txCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool bleDropped = false;

/*
 * BLE writes are only COPIED into this queue inside the NimBLE callback.
 * All real work (LittleFS writes, UART to the RP2040, notifications) runs in
 * loop(). Doing that work inside the callback blocked the BLE host task, which
 * is what made the link drop and reconnect in the middle of a sync.
 */
#define BLE_RX_MAX       260
#define BLE_RX_QUEUE_LEN 40

struct BlePacket {
    uint16_t len;
    uint8_t data[BLE_RX_MAX];
};

static QueueHandle_t bleRxQueue = nullptr;
volatile uint32_t bleRxOverflow = 0;

// ============================================================
// Sync state
// ============================================================

bool syncActive = false;
uint16_t syncCardCount = 0;
uint16_t receivedCards = 0;

File cardsTempFile;
File refsFile;

// ============================================================
// Current card / display state
// ============================================================

struct Card {
    uint32_t frontImg;
    uint32_t backImg;
    String folder;
    String front;
    String back;
};

uint16_t currentCard = 0;
bool currentBack = false;

// ============================================================
// Image receive state
// ============================================================

File imageFile;

uint32_t imageId = 0;
uint16_t imageW = 0;
uint16_t imageH = 0;
uint32_t imageLen = 0;
uint32_t imageReceived = 0;
uint8_t imageFormat = IMAGE_FORMAT_MONO;
uint8_t imageChecksum = 0;
bool receivingImage = false;

// ============================================================
// Helpers
// ============================================================

uint16_t u16le(const uint8_t* p) {
    return (uint16_t)p[0] |
           ((uint16_t)p[1] << 8);
}

uint32_t u32le(const uint8_t* p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

void putU16(uint8_t* p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

void putU32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

String imagePath(uint32_t id) {
    char path[32];
    snprintf(path, sizeof(path), "/i%08lX.bin", (unsigned long)id);
    return String(path);
}

String imageTempPath(uint32_t id) {
    char path[32];
    snprintf(path, sizeof(path), "/i%08lX.tmp", (unsigned long)id);
    return String(path);
}

// ============================================================
// BLE notifications
// ============================================================

void sendStatus(uint8_t refType, uint8_t status) {
    if (!txCharacteristic || !bleConnected)
        return;

    uint8_t packet[3] = {0x80, refType, status};
    txCharacteristic->setValue(packet, sizeof(packet));
    txCharacteristic->notify();
}

// GET_INFO: [0x82][version][maxW u16][maxH u16][maxImg u32][maxText u8][flags]
// flags bit0 = RGB565 supported
void sendInfo() {
    if (!txCharacteristic || !bleConnected)
        return;

    uint8_t p[12];
    p[0] = 0x82;
    p[1] = 2; // protocol v2
    p[2] = MAX_IMG_W & 0xFF;
    p[3] = (MAX_IMG_W >> 8) & 0xFF;
    p[4] = MAX_IMG_H & 0xFF;
    p[5] = (MAX_IMG_H >> 8) & 0xFF;
    putU32(p + 6, (uint32_t)MAX_IMAGE_BYTES);
    p[10] = MAX_TEXT;
    p[11] = 0x01; // RGB565 supported

    txCharacteristic->setValue(p, sizeof(p));
    txCharacteristic->notify();
}

void sendStorage() {
    if (!txCharacteristic || !bleConnected)
        return;

    uint8_t p[9];
    p[0] = 0x81;
    putU32(p + 1, (uint32_t)LittleFS.totalBytes());
    putU32(p + 5, (uint32_t)LittleFS.usedBytes());

    txCharacteristic->setValue(p, sizeof(p));
    txCharacteristic->notify();
}

// ============================================================
// Deck helpers
// ============================================================

void countCards() {
    syncCardCount = 0;

    File f = LittleFS.open("/cards.txt", "r");
    if (!f)
        return;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length())
            syncCardCount++;
    }

    f.close();
}

bool parseCardLine(const String& line, Card& c) {
    int p1 = line.indexOf('\t');
    if (p1 < 0) return false;

    int p2 = line.indexOf('\t', p1 + 1);
    if (p2 < 0) return false;

    int p3 = line.indexOf('\t', p2 + 1);
    if (p3 < 0) return false;

    int p4 = line.indexOf('\t', p3 + 1);

    c.frontImg = strtoul(line.substring(0, p1).c_str(), nullptr, 16);
    c.backImg = strtoul(line.substring(p1 + 1, p2).c_str(), nullptr, 16);

    if (p4 >= 0) {
        // New stored order: frontImg, backImg, folder, front, back
        c.folder = line.substring(p2 + 1, p3);
        c.front = line.substring(p3 + 1, p4);
        c.back = line.substring(p4 + 1);
    } else {
        // Old stored order: frontImg, backImg, front, back
        c.folder = "OllO";
        c.front = line.substring(p2 + 1, p3);
        c.back = line.substring(p3 + 1);
    }

    if (c.folder.length() == 0)
        c.folder = "OllO";

    if (c.folder.length() > MAX_FOLDER_NAME)
        c.folder = c.folder.substring(0, MAX_FOLDER_NAME);

    return true;
}

bool readCard(uint16_t index, Card& card) {
    File f = LittleFS.open("/cards.txt", "r");
    if (!f)
        return false;

    uint16_t n = 0;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.endsWith("\r"))
            line.remove(line.length() - 1);
        if (line.length() == 0)
            continue;

        if (n == index) {
            f.close();
            return parseCardLine(line, card);
        }

        n++;
    }

    f.close();
    return false;
}

// ============================================================
// Image references / cleanup
// ============================================================

void recordImageReference(uint32_t id) {
    if (!id || !refsFile)
        return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%08lX\n", (unsigned long)id);
    refsFile.print(buf);
}

bool imageIsReferenced(uint32_t id) {
    if (!LittleFS.exists("/sync_refs.txt"))
        return false;

    File f = LittleFS.open("/sync_refs.txt", "r");
    if (!f)
        return false;

    char wanted[16];
    snprintf(wanted, sizeof(wanted), "%08lX", (unsigned long)id);

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.equalsIgnoreCase(wanted)) {
            f.close();
            return true;
        }
    }

    f.close();
    return false;
}

void cleanupUnusedImages() {
    File root = LittleFS.open("/");
    if (!root)
        return;

    File file = root.openNextFile();

    while (file) {
        String name = file.name();
        file.close();

        if (name.startsWith("/i") && name.endsWith(".bin")) {
            String hex = name.substring(2, name.length() - 4);
            uint32_t id = strtoul(hex.c_str(), nullptr, 16);
            if (!imageIsReferenced(id))
                LittleFS.remove(name);
        }

        file = root.openNextFile();
    }

    root.close();
}

// ============================================================
// Display pipeline
//
// The ESP32 renders the whole UI with LVGL into a 1-bit 640x480 framebuffer (black/white,
// see ollo_frame.h) and streams it to the RP2040, which only shows it.
// ============================================================

#define DISPLAY_W OLLO_FRAME_W
#define DISPLAY_H OLLO_FRAME_H

/* ---- UI layout (tune these if the glasses crop the edges of the picture) ---- */
// (values were tuned for 320x240; UI_SCALE doubles them for the 640x480 screen)
#define UI_SCALE (DISPLAY_W / 320)
#define FONT_SMALL  (&lv_font_montserrat_28)
#define FONT_MEDIUM (&lv_font_montserrat_32)
#define FONT_LARGE  (&lv_font_montserrat_40)
static const int SAFE_X         = 14 * UI_SCALE;   // left/right margin in pixels
static const int SAFE_TOP       = 10 * UI_SCALE;   // top margin
static const int SAFE_BOTTOM    = 10 * UI_SCALE;   // bottom margin
static const int HEADER_H       = 24 * UI_SCALE;   // folder name + progress bar row
static const int BAR_W          = 44 * UI_SCALE;   // progress bar width
static const int BAR_H          = 4 * UI_SCALE;    // progress bar height
static const int IMAGE_HEADER_Y = 38 * UI_SCALE;   // SAFE_TOP + HEADER_H + 4
static const int IMAGE_FOOTER_H = 52 * UI_SCALE;

#define LV_BUF_LINES 20
alignas(4) static uint8_t lvDrawBuf[DISPLAY_W * LV_BUF_LINES * 2];   // RGB565

static uint8_t* frameBuf = nullptr;      // packed 1-bit pixels, OLLO_ROW_BYTES per row
static lv_display_t* lvDisp = nullptr;
static lv_obj_t* uiRoot = nullptr;

/* LVGL hands us finished RGB565 strips; threshold them to black/white. */
static void lvFlush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    const int w = area->x2 - area->x1 + 1;
    const uint32_t stride = lv_draw_buf_width_to_stride(w, LV_COLOR_FORMAT_RGB565);

    for (int y = area->y1; y <= area->y2; y++) {
        const uint16_t* src = (const uint16_t*)(px_map + (uint32_t)(y - area->y1) * stride);
        if (y >= 0 && y < DISPLAY_H) {
            for (int x = 0; x < w; x++) {
                const int fx = area->x1 + x;
                if (fx >= 0 && fx < DISPLAY_W)
                    olloPutPixel(frameBuf, fx, y, olloRgb565ToGray(src[x]) >= 128);
            }
        }
    }
    lv_display_flush_ready(disp);
}

void initUiRenderer() {
    frameBuf = (uint8_t*)malloc(OLLO_FRAME_BYTES);
    if (!frameBuf) {
        Serial.println("Frame buffer alloc FAILED");
        while (true) delay(1000);
    }
    memset(frameBuf, OLLO_FILL_WHITE, OLLO_FRAME_BYTES);   // start all white

    lv_init();
    lv_tick_set_cb([]() -> uint32_t { return (uint32_t)millis(); });

    lvDisp = lv_display_create(DISPLAY_W, DISPLAY_H);
    lv_display_set_color_format(lvDisp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(lvDisp, lvDrawBuf, nullptr, sizeof(lvDrawBuf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(lvDisp, lvFlush);

    uiRoot = lv_screen_active();
    lv_obj_set_style_bg_color(uiRoot, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(uiRoot, LV_OPA_COVER, LV_PART_MAIN);
}

static lv_obj_t* makeLabel(
    const char* text,
    const lv_font_t* font,
    int width,
    int height,
    lv_align_t align,
    int x,
    int y,
    lv_label_long_mode_t mode,
    bool center
) {
    lv_obj_t* label = lv_label_create(uiRoot);
    lv_label_set_long_mode(label, mode);
    lv_label_set_text(label, text ? text : "");
    lv_obj_set_width(label, width);
    if (height > 0)
        lv_obj_set_height(label, height);

    if (center)
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_pad_all(label, 0, LV_PART_MAIN);
    lv_obj_align(label, align, x, y);
    return label;
}

/* Builds the card UI and renders it into frameBuf (image area is left white). */
static void renderUi(
    uint16_t cardNo,
    uint16_t total,
    const char* folder,
    const char* text,
    bool hasImage
) {
    lv_obj_clean(uiRoot);
    lv_obj_set_style_bg_color(uiRoot, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(uiRoot, LV_OPA_COVER, LV_PART_MAIN);

    /* ---- Header: folder name (left) + small progress bar (right) ---- */
    char safeFolder[MAX_FOLDER_NAME + 1];
    strncpy(safeFolder, folder && folder[0] ? folder : "OllO", MAX_FOLDER_NAME);
    safeFolder[MAX_FOLDER_NAME] = '\0';

    const int folderW = DISPLAY_W - (2 * SAFE_X) - BAR_W - 12;
    makeLabel(safeFolder, FONT_SMALL, folderW, HEADER_H - 4,
              LV_ALIGN_TOP_LEFT, SAFE_X, SAFE_TOP, LV_LABEL_LONG_DOT, false);

    lv_obj_t* bar = lv_bar_create(uiRoot);
    lv_obj_set_size(bar, BAR_W, BAR_H);
    lv_obj_align(bar, LV_ALIGN_TOP_RIGHT, -SAFE_X, SAFE_TOP + 6);
    lv_bar_set_range(bar, 0, total > 0 ? total : 1);
    lv_bar_set_value(bar, total > 0 ? min((int)cardNo, (int)total) : 0, LV_ANIM_OFF);

    lv_obj_set_style_radius(bar, BAR_H / 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, UI_SCALE, LV_PART_MAIN);   // 1-bit screen: outline instead of a gray track
    lv_obj_set_style_border_color(bar, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);

    lv_obj_set_style_radius(bar, BAR_H / 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x000000), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);

    lv_obj_t* separator = lv_obj_create(uiRoot);
    lv_obj_remove_style_all(separator);
    lv_obj_set_size(separator, DISPLAY_W - (2 * SAFE_X), UI_SCALE);
    lv_obj_set_style_bg_color(separator, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(separator, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_align(separator, LV_ALIGN_TOP_MID, 0, SAFE_TOP + HEADER_H);

    /* ---- Body ---- */
    if (hasImage) {
        makeLabel(text, FONT_MEDIUM, DISPLAY_W - (2 * SAFE_X),
                  IMAGE_FOOTER_H - SAFE_BOTTOM, LV_ALIGN_BOTTOM_MID, 0, -SAFE_BOTTOM,
                  LV_LABEL_LONG_WRAP, true);
    } else {
        makeLabel(text, FONT_LARGE, DISPLAY_W - (2 * SAFE_X), 0,
                  LV_ALIGN_CENTER, 0, (SAFE_TOP + HEADER_H) / 2,
                  LV_LABEL_LONG_WRAP, true);
    }

    lv_obj_invalidate(uiRoot);   // redraw the whole screen so nothing from the last card survives
    lv_refr_now(lvDisp);
}

// ---------- card images (read from LittleFS, drawn straight into frameBuf) ----------

struct ImageInfo {
    uint16_t w;
    uint16_t h;
    uint8_t format;
    uint32_t dataOffset;
};

/* Opens and validates a stored image. Same file layout rules as the old bridge. */
static bool openImage(uint32_t id, File& f, ImageInfo& info) {
    if (id == 0)
        return false;

    f = LittleFS.open(imagePath(id), "r");
    if (!f)
        return false;

    const uint32_t fileSize = (uint32_t)f.size();
    uint8_t wh[4];
    if (fileSize < 4 || f.read(wh, 4) != 4) {
        f.close();
        return false;
    }

    const uint16_t w = u16le(wh);
    const uint16_t h = u16le(wh + 2);
    const uint32_t monoLen = ((uint32_t)(w + 7) / 8) * h;
    const uint32_t colorLen = (uint32_t)w * h * 2UL;
    const uint32_t remaining = fileSize - 4;

    uint8_t format;
    bool hasFormatHeader;

    if (remaining == colorLen + 1 && w > 0 && h > 0 && w <= MAX_COLOR_IMG_W && h <= MAX_COLOR_IMG_H) {
        format = IMAGE_FORMAT_RGB565;
        hasFormatHeader = true;
    } else if (remaining == monoLen + 1 && w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H) {
        format = IMAGE_FORMAT_MONO;
        hasFormatHeader = true;
    } else if (remaining == monoLen && w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H) {
        format = IMAGE_FORMAT_MONO;      // old mono files without a format byte
        hasFormatHeader = false;
    } else {
        f.close();
        return false;
    }

    if (hasFormatHeader) {
        uint8_t stored = 0;
        if (f.read(&stored, 1) != 1 || stored != format) {
            f.close();
            return false;
        }
    }

    info.w = w;
    info.h = h;
    info.format = format;
    info.dataOffset = 4 + (hasFormatHeader ? 1 : 0);
    return true;
}

/* Scales the image to fit between header and caption and draws it into frameBuf. */
static void blitImage(File& f, const ImageInfo& info) {
    const int w = info.w;
    const int h = info.h;
    const bool mono = info.format == IMAGE_FORMAT_MONO;

    const int areaH = DISPLAY_H - IMAGE_FOOTER_H - IMAGE_HEADER_Y;
    const float scaleW = (float)(DISPLAY_W - 20) / (float)w;
    const float scaleH = (float)areaH / (float)h;
    const float maxScale = mono ? 1.0f : (float)UI_SCALE;   // colour images are at most 320x240, so enlarge them
    const float scale = min(maxScale, min(scaleW, scaleH));
    const int targetW = max(1, (int)(w * scale));
    const int targetH = max(1, (int)(h * scale));
    const int x0 = (DISPLAY_W - targetW) / 2;
    const int y0 = IMAGE_HEADER_Y + (areaH - targetH) / 2;

    static uint8_t rowBuf[MAX_COLOR_IMG_W * 2];   // largest row: 320 RGB565 px = 640 B (mono max 80 B)
    const uint32_t rowBytes = mono ? (uint32_t)((w + 7) / 8) : (uint32_t)w * 2UL;
    int lastSy = -1;

    for (int dy = 0; dy < targetH; dy++) {
        const int sy = min(h - 1, (dy * h) / targetH);
        if (sy != lastSy) {
            f.seek(info.dataOffset + (uint32_t)sy * rowBytes);
            if (f.read(rowBuf, rowBytes) != (int)rowBytes)
                return;
            lastSy = sy;
        }

        const int fy = y0 + dy;
        if (fy < 0 || fy >= DISPLAY_H)
            continue;

        for (int dx = 0; dx < targetW; dx++) {
            const int fx = x0 + dx;
            if (fx < 0 || fx >= DISPLAY_W)
                continue;
            const int sx = min(w - 1, (dx * w) / targetW);
            bool white;
            if (mono) {
                white = (rowBuf[sx >> 3] & (0x80 >> (sx & 7))) != 0;
            } else {
                // colour photo -> black/white with a 4x4 ordered dither (looks like grays)
                static const uint8_t bayer4[4][4] = {
                    { 0,  8,  2, 10}, {12,  4, 14,  6},
                    { 3, 11,  1,  9}, {15,  7, 13,  5}};
                const uint16_t px = (uint16_t)rowBuf[sx * 2] | ((uint16_t)rowBuf[sx * 2 + 1] << 8);
                white = (int)olloRgb565ToGray(px) > (int)bayer4[fy & 3][fx & 3] * 16 + 8;
            }
            olloPutPixel(frameBuf, fx, fy, white);
        }
    }
}

// ---------- sending the frame to the RP2040 ----------

static void sendFrameToRP() {
    static uint8_t pkt[3 + OLLO_ROW_BYTES];
    static uint8_t enc[OLLO_ROW_BYTES];

    const uint8_t header[3] = {OLLO_MAGIC0, OLLO_MAGIC1, OLLO_CMD_FRAME};
    RPSerial.write(header, 3);

    uint8_t sum = 0;
    for (int y = 0; y < DISPLAY_H; y++) {
        const uint8_t* row = frameBuf + (size_t)y * OLLO_ROW_BYTES;
        for (int x = 0; x < OLLO_ROW_BYTES; x++)
            sum += row[x];

        const size_t n = olloRleEncodeRow(row, enc);
        if (n) {
            pkt[0] = OLLO_ROW_RLE;
            pkt[1] = (uint8_t)(n & 0xFF);
            pkt[2] = (uint8_t)(n >> 8);
            memcpy(pkt + 3, enc, n);
            RPSerial.write(pkt, 3 + n);
        } else {
            pkt[0] = OLLO_ROW_RAW;
            memcpy(pkt + 1, row, OLLO_ROW_BYTES);
            RPSerial.write(pkt, 1 + OLLO_ROW_BYTES);
        }
    }

    RPSerial.write(sum);
    RPSerial.flush();
}

void sendCardToRP(uint16_t index, bool back) {
    Card card;
    if (!readCard(index, card))
        return;

    const uint32_t imageId = back ? card.backImg : card.frontImg;
    const String& text = back ? card.back : card.front;

    File cardImage;
    ImageInfo info;
    const bool hasImage = openImage(imageId, cardImage, info);

    renderUi(index + 1, syncCardCount, card.folder.c_str(), text.c_str(), hasImage);

    if (hasImage) {
        blitImage(cardImage, info);
        cardImage.close();
    }

    sendFrameToRP();
}

void showCurrentCard() {
    if (syncCardCount == 0) {
        renderUi(0, 0, "OllO", "No cards yet", false);
        sendFrameToRP();
        return;
    }

    sendCardToRP(currentCard, currentBack);
}

// ============================================================
// Sync abort
// ============================================================

void abortSync() {
    if (!syncActive && !receivingImage)
        return;

    if (cardsTempFile) cardsTempFile.close();
    if (refsFile) refsFile.close();

    if (receivingImage) {
        if (imageFile)
            imageFile.close();

        LittleFS.remove(imageTempPath(imageId));
        receivingImage = false;
    }

    if (syncActive) {
        LittleFS.remove("/cards.new");
        LittleFS.remove("/sync_refs.txt");
    }

    syncActive = false;
    countCards();

    if (currentCard >= syncCardCount)
        currentCard = 0;

    Serial.printf("abortSync: cards on flash = %u\n", syncCardCount);
}

// ============================================================
// BLE callbacks
// ============================================================

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(
        NimBLEServer* server,
        NimBLEConnInfo& connInfo
    ) override {
        bleConnected = true;
        Serial.println("BLE connected");
    }

    void onDisconnect(
        NimBLEServer* server,
        NimBLEConnInfo& connInfo,
        int reason
    ) override {
        bleConnected = false;
        bleDropped = true;
        Serial.println("BLE disconnected");
        NimBLEDevice::startAdvertising();
    }
};

// ============================================================
// BEGIN_SYNC
// ============================================================

void beginSync(uint16_t cardCount) {
    if (syncActive) {
        if (cardsTempFile) cardsTempFile.close();
        if (refsFile) refsFile.close();
        syncActive = false;
    }

    if (cardCount > MAX_CARDS) {
        sendStatus(0x01, 1);
        return;
    }

    LittleFS.remove("/cards.new");
    LittleFS.remove("/sync_refs.txt");

    cardsTempFile = LittleFS.open("/cards.new", "w");
    refsFile = LittleFS.open("/sync_refs.txt", "w");

    if (!cardsTempFile || !refsFile) {
        if (cardsTempFile) cardsTempFile.close();
        if (refsFile) refsFile.close();
        sendStatus(0x01, 3);
        return;
    }

    syncCardCount = cardCount;
    receivedCards = 0;
    syncActive = true;

    sendStatus(0x01, 0);
    Serial.printf("BEGIN_SYNC cards=%u\n", cardCount);
}

// ============================================================
// CARD
// ============================================================

void receiveCard(const uint8_t* p, size_t len) {
    if (!syncActive || (len != 12 && len < 13)) {
        sendStatus(0x02, 1);
        return;
    }

    const uint16_t index = u16le(p);
    const uint32_t frontImg = u32le(p + 2);
    const uint32_t backImg = u32le(p + 6);

    /* Old format: [index 2][frontImg 4][backImg 4][frontLen 1][backLen 1] */
    if (len == 12 + p[10] + p[11]) {
        const uint8_t frontLen = p[10];
        const uint8_t backLen = p[11];

        if (index >= syncCardCount || frontLen > MAX_TEXT || backLen > MAX_TEXT) {
            sendStatus(0x02, 1);
            return;
        }

        String front;
        String back;
        for (uint8_t i = 0; i < frontLen; i++)
            front += (char)p[12 + i];
        for (uint8_t i = 0; i < backLen; i++)
            back += (char)p[12 + frontLen + i];

        cardsTempFile.printf(
            "%08lX\t%08lX\t%s\t%s\n",
            (unsigned long)frontImg,
            (unsigned long)backImg,
            front.c_str(),
            back.c_str()
        );

        if (!cardsTempFile) {
            sendStatus(0x02, 3);
            return;
        }

        if (frontImg) recordImageReference(frontImg);
        if (backImg) recordImageReference(backImg);

        receivedCards++;
        sendStatus(0x02, 0);
        return;
    }

    /* New format: [index 2][frontImg 4][backImg 4][folderLen 1][frontLen 1][backLen 1][folder][front][back] */
    if (len < 13) {
        sendStatus(0x02, 1);
        return;
    }

    const uint8_t folderLen = p[10];
    const uint8_t frontLen = p[11];
    const uint8_t backLen = p[12];

    if (index >= syncCardCount ||
        folderLen > MAX_FOLDER_NAME ||
        frontLen > MAX_TEXT ||
        backLen > MAX_TEXT ||
        13 + folderLen + frontLen + backLen != len) {
        sendStatus(0x02, 1);
        return;
    }

    String folder;
    String front;
    String back;

    for (uint8_t i = 0; i < folderLen; i++)
        folder += (char)p[13 + i];

    for (uint8_t i = 0; i < frontLen; i++)
        front += (char)p[13 + folderLen + i];

    for (uint8_t i = 0; i < backLen; i++)
        back += (char)p[13 + folderLen + frontLen + i];

    if (folder.length() == 0)
        folder = "OllO";

    cardsTempFile.printf(
        "%08lX\t%08lX\t%s\t%s\t%s\n",
        (unsigned long)frontImg,
        (unsigned long)backImg,
        folder.c_str(),
        front.c_str(),
        back.c_str()
    );

    if (!cardsTempFile) {
        sendStatus(0x02, 3);
        return;
    }

    if (frontImg) recordImageReference(frontImg);
    if (backImg) recordImageReference(backImg);

    receivedCards++;
    sendStatus(0x02, 0);

    Serial.printf("CARD %u/%u folder=%s\n", receivedCards, syncCardCount, folder.c_str());
}

// ============================================================
// IMG_BEGIN
// ============================================================

void beginImage(const uint8_t* p, size_t len) {
    if (!syncActive || (len != 12 && len != 13)) {
        sendStatus(0x03, 1);
        return;
    }

    const uint32_t id = u32le(p);
    const uint16_t w = u16le(p + 4);
    const uint16_t h = u16le(p + 6);
    const uint32_t dataLen = u32le(p + 8);
    const uint8_t format = (len == 13) ? p[12] : IMAGE_FORMAT_MONO;

    uint32_t expected = 0;
    uint32_t maxBytes = 0;

    if (format == IMAGE_FORMAT_MONO) {
        expected = ((uint32_t)(w + 7) / 8) * h;
        maxBytes = MAX_MONO_IMAGE_BYTES;
        if (w == 0 || h == 0 || w > MAX_IMG_W || h > MAX_IMG_H)
            expected = 0;
    } else if (format == IMAGE_FORMAT_RGB565) {
        if (w <= MAX_COLOR_IMG_W && h <= MAX_COLOR_IMG_H)
            expected = (uint32_t)w * h * 2UL;
        maxBytes = MAX_COLOR_IMAGE_BYTES;
    } else {
        sendStatus(0x03, 1);
        return;
    }

    if (id == 0 || expected == 0 || dataLen != expected || dataLen > maxBytes) {
        sendStatus(0x03, 1);
        return;
    }

    String finalPath = imagePath(id);

    if (LittleFS.exists(finalPath)) {
        sendStatus(0x03, 2);
        Serial.printf("IMG %08lX already exists\n", (unsigned long)id);
        return;
    }

    const size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
    if (freeBytes < dataLen + 32) {
        sendStatus(0x03, 3);
        return;
    }

    if (receivingImage) {
        if (imageFile) imageFile.close();
        receivingImage = false;
    }

    const String tempPath = imageTempPath(id);
    LittleFS.remove(tempPath);

    imageFile = LittleFS.open(tempPath, "w");
    if (!imageFile) {
        sendStatus(0x03, 3);
        return;
    }

    // New image header: width u16, height u16, format u8.
    uint8_t header[5];
    putU16(header, w);
    putU16(header + 2, h);
    header[4] = format;

    if (imageFile.write(header, 5) != 5) {
        imageFile.close();
        LittleFS.remove(tempPath);
        sendStatus(0x03, 3);
        return;
    }

    imageId = id;
    imageW = w;
    imageH = h;
    imageLen = dataLen;
    imageReceived = 0;
    imageFormat = format;
    imageChecksum = 0;
    receivingImage = true;

    sendStatus(0x03, 0);

    Serial.printf(
        "IMG_BEGIN id=%08lX %ux%u len=%lu format=%u\n",
        (unsigned long)id,
        w,
        h,
        (unsigned long)dataLen,
        (unsigned)format
    );
}

// ============================================================
// IMG_CHUNK
// ============================================================

void receiveImageChunk(const uint8_t* p, size_t len) {
    if (!receivingImage || len < 4)
        return;

    const uint32_t offset = u32le(p);
    const uint8_t* data = p + 4;
    const size_t dataLen = len - 4;

    if (offset != imageReceived || imageReceived + dataLen > imageLen) {
        Serial.printf(
            "IMG offset error: got=%lu expected=%lu\n",
            (unsigned long)offset,
            (unsigned long)imageReceived
        );

        if (imageFile) imageFile.close();
        LittleFS.remove(imageTempPath(imageId));
        receivingImage = false;
        return;
    }

    if (imageFile.write(data, dataLen) != dataLen) {
        if (imageFile) imageFile.close();
        LittleFS.remove(imageTempPath(imageId));
        receivingImage = false;
        return;
    }

    for (size_t i = 0; i < dataLen; i++)
        imageChecksum += data[i];

    imageReceived += dataLen;
}

// ============================================================
// IMG_END
// ============================================================

void endImage(const uint8_t* p, size_t len) {
    if (!receivingImage || len != 1) {
        sendStatus(0x05, 1);
        return;
    }

    const uint8_t expectedChecksum = p[0];
    const bool good =
        imageReceived == imageLen &&
        imageChecksum == expectedChecksum;

    if (imageFile)
        imageFile.close();

    const String tempPath = imageTempPath(imageId);
    const String finalPath = imagePath(imageId);

    if (!good) {
        LittleFS.remove(tempPath);
        receivingImage = false;
        Serial.println("IMG checksum/length error");
        sendStatus(0x05, 1);
        return;
    }

    LittleFS.remove(finalPath);

    if (!LittleFS.rename(tempPath, finalPath)) {
        LittleFS.remove(tempPath);
        receivingImage = false;
        sendStatus(0x05, 3);
        return;
    }

    receivingImage = false;
    Serial.printf("IMG complete %08lX format=%u\n", (unsigned long)imageId, (unsigned)imageFormat);
    sendStatus(0x05, 0);
}

// ============================================================
// END_SYNC
// ============================================================

void endSync() {
    if (!syncActive) {
        sendStatus(0x06, 1);
        return;
    }

    if (receivedCards != syncCardCount) {
        if (cardsTempFile) cardsTempFile.close();
        if (refsFile) refsFile.close();

        syncActive = false;
        LittleFS.remove("/cards.new");
        LittleFS.remove("/sync_refs.txt");
        countCards();

        sendStatus(0x06, 1);
        Serial.println("END_SYNC card count mismatch");
        return;
    }

    if (cardsTempFile) cardsTempFile.close();
    if (refsFile) refsFile.close();

    LittleFS.remove("/cards.old");
    if (LittleFS.exists("/cards.txt"))
        LittleFS.rename("/cards.txt", "/cards.old");

    if (!LittleFS.rename("/cards.new", "/cards.txt")) {
        LittleFS.remove("/cards.txt");
        if (LittleFS.exists("/cards.old"))
            LittleFS.rename("/cards.old", "/cards.txt");

        syncActive = false;
        countCards();
        sendStatus(0x06, 1);
        return;
    }

    LittleFS.remove("/cards.old");
    cleanupUnusedImages();
    LittleFS.remove("/sync_refs.txt");

    syncActive = false;
    currentCard = 0;
    currentBack = false;

    sendStatus(0x06, 0);
    sendStorage();

    Serial.printf("SYNC COMPLETE: %u cards\n", syncCardCount);
    delay(50);
    showCurrentCard();
}

// ============================================================
// BLE packet dispatcher
// ============================================================

void handleBLEPacket(const uint8_t* data, size_t len) {
    if (!data || len < 1)
        return;

    const uint8_t type = data[0];
    const uint8_t* payload = data + 1;
    const size_t payloadLen = len - 1;

    switch (type) {
        case 0x01:
            if (payloadLen != 2) { sendStatus(0x01, 1); return; }
            beginSync(u16le(payload));
            break;

        case 0x02:
            receiveCard(payload, payloadLen);
            break;

        case 0x03:
            beginImage(payload, payloadLen);
            break;

        case 0x04:
            receiveImageChunk(payload, payloadLen);
            break;

        case 0x05:
            endImage(payload, payloadLen);
            break;

        case 0x06:
            if (payloadLen != 0) { sendStatus(0x06, 1); return; }
            endSync();
            break;

        case 0x07:
            sendStorage();
            break;

        case 0x08:
            sendInfo();
            break;

        default:
            Serial.printf("Unknown BLE packet: 0x%02X\n", type);
            break;
    }
}

// ============================================================
// BLE write callback
// ============================================================

class WriteCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(
        NimBLECharacteristic* characteristic,
        NimBLEConnInfo& connInfo
    ) override {
        std::string value = characteristic->getValue();
        if (value.empty() || value.size() > BLE_RX_MAX || !bleRxQueue)
            return;

        BlePacket pkt;
        pkt.len = (uint16_t)value.size();
        memcpy(pkt.data, value.data(), value.size());

        // Never do real work here - just hand the packet to loop().
        if (xQueueSend(bleRxQueue, &pkt, pdMS_TO_TICKS(30)) != pdTRUE)
            bleRxOverflow = bleRxOverflow + 1;
    }
};

void processBleQueue() {
    if (!bleRxQueue)
        return;

    BlePacket pkt;
    while (xQueueReceive(bleRxQueue, &pkt, 0) == pdTRUE)
        handleBLEPacket(pkt.data, pkt.len);
}

// ============================================================
// Card navigation commands
// ============================================================

String rpLine;

void handleRPCommand(const String& command) {
    Serial.printf(
        "cmd %s syncActive=%d cards=%u cur=%u back=%d\n",
        command.c_str(),
        (int)syncActive,
        syncCardCount,
        currentCard,
        (int)currentBack
    );

    if (command == "READY") {   // the RP2040 just booted: send it the current screen
        if (!syncActive)
            showCurrentCard();
        return;
    }

    if (syncActive || syncCardCount == 0)
        return;

    if (command == "NEXT") {
        if (currentCard + 1 < syncCardCount)
            currentCard++;
        currentBack = false;
        showCurrentCard();
    } else if (command == "PREV") {
        if (currentCard > 0)
            currentCard--;
        currentBack = false;
        showCurrentCard();
    } else if (command == "FLIP") {
        currentBack = !currentBack;
        showCurrentCard();
    }
}

void readRPCommands() {
    while (RPSerial.available()) {
        char c = RPSerial.read();

        if (c == '\r')
            continue;

        if (c == '\n') {
            rpLine.trim();
            if (rpLine.length())
                handleRPCommand(rpLine);
            rpLine = "";
        } else if (rpLine.length() < 32) {
            rpLine += c;
        }
    }
}

// ============================================================
// Touch pads
// ============================================================

#define DOUBLE_TAP_MS      400
#define TOUCH_COOLDOWN_MS  150

struct TapPad {
    uint8_t pin;
    bool last;
    uint8_t taps;
    uint32_t lastTapAt;
};

TapPad padBack  = { TOUCH_BACK_PIN,  false, 0, 0 };
TapPad padFront = { TOUCH_FRONT_PIN, false, 0, 0 };

enum PadResult { PAD_NONE, PAD_SINGLE, PAD_DOUBLE };
uint32_t touchCooldownUntil = 0;

PadResult pollPad(TapPad& p, uint32_t now) {
    const bool down = digitalRead(p.pin);
    const bool rise = down && !p.last;
    p.last = down;

    if (rise) {
        p.taps++;
        p.lastTapAt = now;
        if (p.taps >= 2) {
            p.taps = 0;
            return PAD_DOUBLE;
        }
    }

    if (p.taps == 1 && (now - p.lastTapAt) > DOUBLE_TAP_MS) {
        p.taps = 0;
        return PAD_SINGLE;
    }

    return PAD_NONE;
}

void readTouch() {
    const uint32_t now = millis();
    const PadResult f = pollPad(padFront, now);
    const PadResult b = pollPad(padBack, now);

    if ((int32_t)(now - touchCooldownUntil) < 0)
        return;

    if (f == PAD_DOUBLE) {
        handleRPCommand("NEXT");
    } else if (f == PAD_SINGLE) {
        handleRPCommand("FLIP");
    } else if (b == PAD_DOUBLE) {
        handleRPCommand("PREV");
    } else if (b == PAD_SINGLE) {
        handleRPCommand("FLIP");
    } else {
        return;
    }

    touchCooldownUntil = now + TOUCH_COOLDOWN_MS;
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("================================");
    Serial.println("Ollo ESP32-S3");
    Serial.println("BLE deck manager / protocol v2");
    Serial.println("================================");

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount FAILED");
        while (true) delay(1000);
    }

    Serial.printf(
        "LittleFS: %u / %u bytes used\n",
        (unsigned)LittleFS.usedBytes(),
        (unsigned)LittleFS.totalBytes()
    );

    countCards();
    Serial.printf("Cards on flash: %u\n", syncCardCount);

    initUiRenderer();

    RPSerial.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX,
        UART_TX
    );

    pinMode(TOUCH_BACK_PIN, INPUT);
    pinMode(TOUCH_FRONT_PIN, INPUT);
    padBack.last = digitalRead(TOUCH_BACK_PIN);
    padFront.last = digitalRead(TOUCH_FRONT_PIN);

    bleRxQueue = xQueueCreate(BLE_RX_QUEUE_LEN, sizeof(BlePacket));
    if (!bleRxQueue)
        Serial.println("BLE rx queue creation FAILED");

    NimBLEDevice::init("Ollo");

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    NimBLEService* service = bleServer->createService(SERVICE_UUID);

    txCharacteristic = service->createCharacteristic(
        NOTIFY_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    NimBLECharacteristic* rxCharacteristic = service->createCharacteristic(
        WRITE_UUID,
        NIMBLE_PROPERTY::WRITE_NR
    );

    rxCharacteristic->setCallbacks(new WriteCallbacks());
    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setName("Ollo");
    advertising->start();

    Serial.println("BLE advertising as Ollo");

    delay(500);
    showCurrentCard();
}

void loop() {
    if (bleDropped) {
        bleDropped = false;
        if (bleRxQueue)
            xQueueReset(bleRxQueue);   // drop anything left from the old link
        abortSync();
    }

    processBleQueue();

    if (bleRxOverflow) {
        Serial.printf("BLE rx queue overflow (%lu packets dropped)\n", (unsigned long)bleRxOverflow);
        bleRxOverflow = 0;
    }

    readRPCommands();
    readTouch();
    delay(1);
}
