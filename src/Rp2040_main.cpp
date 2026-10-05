/*
 * OllO - RP2040 display/UI driver
 *
 * Arduino-Pico + PicoDVI + LVGL 9.6
 *
 * The RP2040 intentionally does NOT keep a full RGB framebuffer.
 * LVGL renders normal UI into a small partial RGB565 draw buffer.
 * RGB565 photos are streamed from the ESP32 one row at a time.
 * 1-bit photos continue to use the existing compact full image buffer.
 *
 * ESP32 -> RP2040 UART protocol:
 *   Card metadata:
 *     F/B\tcardNo\ttotal\tfolder\ttext\n
 *     The old F/B\tcardNo\ttotal\ttext format is also accepted.
 *
 *   No image:
 *     N\n
 *   Image:
 *     I\tw\th\tlen\tformat\n
 *     + len raw bytes
 *     + 1 checksum byte
 *
 *     format 0 = 1-bit packed, row-major, MSB first
 *     format 1 = RGB565 little-endian, row-major
 *
 * RP2040 -> ESP32:
 *   NEXT\n
 *   PREV\n
 *   FLIP\n
 */

#include <Arduino.h>
#include <PicoDVI.h>
#include <lvgl.h>

// ============================================================
// DVI
// ============================================================

const struct dvi_serialiser_cfg my_dvi_cfg = {
    .pio = pio0,
    .sm_tmds = {0, 1, 2},
    .pins_tmds = {2, 4, 6},
    .pins_clk = 8,
    .invert_diffpairs = false
};

DVIGFX8 display(
    DVI_RES_320x240p60,
    false,
    my_dvi_cfg
);

// ============================================================
// UART
// ============================================================

#define UART_TX_PIN 12
#define UART_RX_PIN 13
#define UART_BAUD 230400

// ESP GPIO4 -> RP GP13 RX
// ESP GPIO5 <- RP GP12 TX

// ============================================================
// Display / protocol settings
// ============================================================

static const int DISPLAY_W = 320;
static const int DISPLAY_H = 240;
static const int MAX_TEXT = 100;
static const int MAX_FOLDER_NAME = 20;

static const int MAX_MONO_W = 640;
static const int MAX_MONO_H = 480;
static const uint32_t MAX_MONO_BYTES = ((MAX_MONO_W + 7) / 8) * MAX_MONO_H;

static const int MAX_COLOR_W = 320;
static const int MAX_COLOR_H = 240;
static const uint32_t MAX_COLOR_BYTES = (uint32_t)MAX_COLOR_W * MAX_COLOR_H * 2UL;

static const int IMAGE_HEADER_Y = 38;   /* SAFE_TOP + HEADER_H + 4 */
static const int IMAGE_FOOTER_H = 52;

#define RGB565_WHITE 0xFFFF
#define RGB565_BLACK 0x0000

// ============================================================
// LVGL
// ============================================================

/* ---- UI layout (tune these if the glasses crop the edges of the picture) ---- */
static const int SAFE_X      = 14;   // left/right margin in pixels (320x240 canvas)
static const int SAFE_TOP    = 10;   // top margin
static const int SAFE_BOTTOM = 10;   // bottom margin
static const int HEADER_H    = 24;   // folder name + progress bar row
static const int BAR_W       = 44;   // progress bar width  (was 88)
static const int BAR_H       = 4;    // progress bar height (was 6 + border)

/* 8 rows * 320 pixels * 1 byte = 2,560 bytes. */
#define LVGL_BUFFER_LINES 8
alignas(4) static uint8_t lvglDrawBuffer[DISPLAY_W * LVGL_BUFFER_LINES];
static lv_display_t* lvglDisplay = nullptr;
static lv_obj_t* uiRoot = nullptr;

/*
 * Palette layout (see initColorPalette):
 *   0        = black
 *   1..216   = 6x6x6 color cube
 *   217..254 = 38-step gray ramp (gray 6..248)
 *   255      = white
 * LVGL renders the UI as 8-bit gray (L8). Mapping gray -> the gray ramp keeps
 * the anti-aliased edges of the font, which is what removes the pixelated look.
 */
static inline uint8_t grayToPalette(uint8_t g) {
    if (g <= 3)   return 0;
    if (g >= 251) return 255;
    int idx = 216 + ((int)g * 39 + 127) / 255;
    if (idx < 217) idx = 217;
    if (idx > 254) idx = 254;
    return (uint8_t)idx;
}

void lvglFlush(
    lv_display_t* disp,
    const lv_area_t* area,
    uint8_t* px_map
) {
    const int32_t width = area->x2 - area->x1 + 1;
    const int32_t height = area->y2 - area->y1 + 1;

    if (width <= 0 || height <= 0) {
        lv_display_flush_ready(disp);
        return;
    }

    const uint32_t stride = lv_draw_buf_width_to_stride(width, LV_COLOR_FORMAT_L8);
    uint8_t* frame = display.getBuffer();

    for (int32_t y = 0; y < height; y++) {
        const int32_t fy = area->y1 + y;
        if (fy < 0 || fy >= DISPLAY_H) continue;
        uint8_t* dst = frame + (fy * DISPLAY_W) + area->x1;
        const uint8_t* src = px_map + (y * stride);
        for (int32_t x = 0; x < width; x++) {
            dst[x] = grayToPalette(src[x]);
        }
    }

    lv_display_flush_ready(disp);
}

void initColorPalette() {
    display.setColor(0, RGB565_BLACK);

    uint8_t index = 1;
    for (int r = 0; r < 6; r++) {
        for (int g = 0; g < 6; g++) {
            for (int b = 0; b < 6; b++) {
                if (index == 255) break;
                const uint8_t red = (uint8_t)((r * 255) / 5);
                const uint8_t green = (uint8_t)((g * 255) / 5);
                const uint8_t blue = (uint8_t)((b * 255) / 5);
                display.setColor(index++, red, green, blue);
            }
        }
    }

    for (; index < 255; index++) {
        const uint8_t gray = (uint8_t)(((index - 216) * 255) / 39);
        display.setColor(index, gray, gray, gray);
    }

    display.setColor(255, RGB565_WHITE);
}

void initLvgl() {
    lv_init();

    /* LVGL reads the real clock, so redraws never depend on loop timing. */
    lv_tick_set_cb([]() -> uint32_t { return (uint32_t)millis(); });

    lvglDisplay = lv_display_create(DISPLAY_W, DISPLAY_H);
    if (!lvglDisplay) {
        Serial.println("LVGL display creation FAILED");
        while (true) delay(1000);
    }

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_L8);
    lv_display_set_buffers(
        lvglDisplay,
        lvglDrawBuffer,
        nullptr,
        sizeof(lvglDrawBuffer),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );
    lv_display_set_flush_cb(lvglDisplay, lvglFlush);

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

void renderUi(
    uint16_t cardNo,
    uint16_t total,
    const char* folder,
    const char* text,
    bool hasImage,
    uint16_t imageW,
    uint16_t imageH,
    uint8_t imageFormat,
    int* outImageX,
    int* outImageY
) {
    (void)imageFormat;

    if (!uiRoot)
        return;

    lv_obj_clean(uiRoot);

    lv_obj_set_style_bg_color(uiRoot, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(uiRoot, LV_OPA_COVER, LV_PART_MAIN);

    /* ---- Header: folder name (left) + small progress bar (right) ---- */
    char safeFolder[MAX_FOLDER_NAME + 1];
    strncpy(safeFolder, folder && folder[0] ? folder : "OllO", MAX_FOLDER_NAME);
    safeFolder[MAX_FOLDER_NAME] = '\0';

    const int folderW = DISPLAY_W - (2 * SAFE_X) - BAR_W - 12;
    makeLabel(
        safeFolder,
        &lv_font_montserrat_14,
        folderW,
        HEADER_H - 4,
        LV_ALIGN_TOP_LEFT,
        SAFE_X,
        SAFE_TOP,
        LV_LABEL_LONG_DOT,
        false
    );

    lv_obj_t* bar = lv_bar_create(uiRoot);
    lv_obj_set_size(bar, BAR_W, BAR_H);
    lv_obj_align(bar, LV_ALIGN_TOP_RIGHT, -SAFE_X, SAFE_TOP + 6);
    lv_bar_set_range(bar, 0, total > 0 ? total : 1);
    lv_bar_set_value(bar, total > 0 ? min((int)cardNo, (int)total) : 0, LV_ANIM_OFF);

    lv_obj_set_style_radius(bar, BAR_H / 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xB4B4B4), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);

    lv_obj_set_style_radius(bar, BAR_H / 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x000000), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);

    lv_obj_t* separator = lv_obj_create(uiRoot);
    lv_obj_remove_style_all(separator);
    lv_obj_set_size(separator, DISPLAY_W - (2 * SAFE_X), 1);
    lv_obj_set_style_bg_color(separator, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(separator, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_align(separator, LV_ALIGN_TOP_MID, 0, SAFE_TOP + HEADER_H);

    if (outImageX) *outImageX = 0;
    if (outImageY) *outImageY = 0;

    /* ---- Body ---- */
    if (hasImage && imageW > 0 && imageH > 0) {
        const int imageAreaTop = IMAGE_HEADER_Y;
        const int imageAreaBottom = DISPLAY_H - IMAGE_FOOTER_H;
        const int imageAreaH = imageAreaBottom - imageAreaTop;

        const float scaleW = (float)(DISPLAY_W - 20) / (float)imageW;
        const float scaleH = (float)imageAreaH / (float)imageH;
        const float scale = min(1.0f, min(scaleW, scaleH));
        const int targetW = max(1, (int)(imageW * scale));
        const int targetH = max(1, (int)(imageH * scale));

        int x = (DISPLAY_W - targetW) / 2;
        int y = imageAreaTop + (imageAreaH - targetH) / 2;

        if (outImageX) *outImageX = x;
        if (outImageY) *outImageY = y;

        /* The picture itself is drawn after the LVGL flush (no image framebuffer). */
        makeLabel(
            text,
            &lv_font_montserrat_16,
            DISPLAY_W - (2 * SAFE_X),
            IMAGE_FOOTER_H - SAFE_BOTTOM,
            LV_ALIGN_BOTTOM_MID,
            0,
            -SAFE_BOTTOM,
            LV_LABEL_LONG_WRAP,
            true
        );
    } else {
        makeLabel(
            text,
            &lv_font_montserrat_20,
            DISPLAY_W - (2 * SAFE_X),
            0,                      /* auto height so the text block is truly centered */
            LV_ALIGN_CENTER,
            0,
            (SAFE_TOP + HEADER_H) / 2,
            LV_LABEL_LONG_WRAP,
            true
        );
    }

    /* Draw right now so the picture that follows is never wiped by a later refresh. */
    lv_refr_now(lvglDisplay);
}

// ============================================================
// Image buffer / receive state
// ============================================================

static uint8_t monoImgBuf[MAX_MONO_BYTES];
static uint16_t colorRowBuf[MAX_COLOR_W];

static bool pendingCardValid = false;
static uint16_t pendingCardNo = 0;
static uint16_t pendingTotal = 0;
static char pendingFolder[MAX_FOLDER_NAME + 1] = "OllO";
static char pendingText[MAX_TEXT + 1] = "";

static uint16_t currentImageW = 0;
static uint16_t currentImageH = 0;
static uint8_t currentImageFormat = 0;

bool waitForBytes(size_t needed, uint32_t timeoutMs) {
    uint32_t start = millis();
    while (Serial1.available() < (int)needed) {
        if (millis() - start > timeoutMs)
            return false;
        delay(1);
    }
    return true;
}


uint8_t rgb565ToPalette(uint16_t pixel) {
    const uint8_t r = (pixel >> 11) & 0x1F;
    const uint8_t g = (pixel >> 5) & 0x3F;
    const uint8_t b = pixel & 0x1F;

    const uint8_t rq = (uint8_t)((r * 5 + 15) / 31);
    const uint8_t gq = (uint8_t)((g * 5 + 31) / 63);
    const uint8_t bq = (uint8_t)((b * 5 + 15) / 31);

    if (rq == 0 && gq == 0 && bq == 0)
        return 0;
    if (rq == 5 && gq == 5 && bq == 5)
        return 255;

    return (uint8_t)(1 + rq * 36 + gq * 6 + bq);
}

void drawColorImageRow(
    int imageX,
    int imageY,
    uint16_t sourceWidth,
    const uint16_t* row,
    uint16_t targetWidth
) {
    uint8_t* frame = display.getBuffer();
    if (imageY < 0 || imageY >= DISPLAY_H)
        return;

    for (uint16_t x = 0; x < targetWidth; x++) {
        const uint16_t srcX = (uint16_t)(((uint32_t)x * sourceWidth) / targetWidth);
        const int dstX = imageX + x;
        if (dstX >= 0 && dstX < DISPLAY_W)
            frame[imageY * DISPLAY_W + dstX] = rgb565ToPalette(row[srcX]);
    }
}

bool receiveImage(
    uint16_t w,
    uint16_t h,
    uint32_t len,
    uint8_t format
) {
    currentImageW = w;
    currentImageH = h;
    currentImageFormat = format;

    const bool mono = format == 0;
    const bool color = format == 1;

    uint32_t expected = 0;

    if (mono) {
        if (w == 0 || h == 0 || w > MAX_MONO_W || h > MAX_MONO_H)
            return false;
        expected = ((uint32_t)(w + 7) / 8) * h;
        if (len != expected || len > MAX_MONO_BYTES)
            return false;
    } else if (color) {
        if (w == 0 || h == 0 || w > MAX_COLOR_W || h > MAX_COLOR_H)
            return false;
        expected = (uint32_t)w * h * 2UL;
        if (len != expected || len > MAX_COLOR_BYTES)
            return false;
    } else {
        return false;
    }

    uint8_t checksum = 0;
    uint32_t start = millis();

    if (color && pendingCardValid) {
        int imageX = 0;
        int imageY = 0;
        renderUi(
            pendingCardNo,
            pendingTotal,
            pendingFolder,
            pendingText,
            true,
            w,
            h,
            1,
            &imageX,
            &imageY
        );

        const int imageAreaH = DISPLAY_H - IMAGE_FOOTER_H - IMAGE_HEADER_Y;
        const float scaleW = (float)(DISPLAY_W - 20) / (float)w;
        const float scaleH = (float)imageAreaH / (float)h;
        const float scale = min(1.0f, min(scaleW, scaleH));
        const uint16_t targetW = (uint16_t)max(1, (int)(w * scale));
        const uint16_t targetH = (uint16_t)max(1, (int)(h * scale));
        const uint32_t rowBytes = (uint32_t)w * 2UL;
        int lastTargetY = -1;

        for (uint16_t y = 0; y < h; y++) {
            size_t got = 0;
            uint32_t rowStart = millis();
            uint8_t* rowBytesPtr = reinterpret_cast<uint8_t*>(colorRowBuf);

            while (got < rowBytes) {
                if (Serial1.available()) {
                    const uint8_t b = Serial1.read();
                    rowBytesPtr[got++] = b;
                    checksum += b;
                    rowStart = millis();
                } else if (millis() - rowStart > 3000) {
                    return false;
                }
            }

            const int targetY = imageY + (int)(((uint32_t)y * targetH) / h);
            if (targetY != lastTargetY) {
                drawColorImageRow(
                    imageX,
                    targetY,
                    w,
                    colorRowBuf,
                    targetW
                );
                lastTargetY = targetY;
            }

            start = millis();
        }
    } else if (mono) {
        uint32_t got = 0;
        while (got < len) {
            if (Serial1.available()) {
                uint8_t b = Serial1.read();
                monoImgBuf[got++] = b;
                checksum += b;
                start = millis();
            } else if (millis() - start > 3000) {
                return false;
            }
        }
    } else {
        uint32_t got = 0;
        while (got < len) {
            if (Serial1.available()) {
                uint8_t b = Serial1.read();
                checksum += b;
                got++;
                start = millis();
            } else if (millis() - start > 3000) {
                return false;
            }
        }
    }

    if (!waitForBytes(1, 3000))
        return false;

    const uint8_t expectedChecksum = Serial1.read();

    if (checksum != expectedChecksum)
        return false;

    if (mono && pendingCardValid) {
        int imageX = 0;
        int imageY = 0;
        renderUi(
            pendingCardNo,
            pendingTotal,
            pendingFolder,
            pendingText,
            true,
            w,
            h,
            0,
            &imageX,
            &imageY
        );

        const int imageAreaH = DISPLAY_H - IMAGE_FOOTER_H - IMAGE_HEADER_Y;
        const float scaleW = (float)(DISPLAY_W - 20) / (float)w;
        const float scaleH = (float)imageAreaH / (float)h;
        const float scale = min(1.0f, min(scaleW, scaleH));
        const int targetW = max(1, (int)(w * scale));
        const int targetH = max(1, (int)(h * scale));
        const int targetX = (DISPLAY_W - targetW) / 2;
        const int targetY = IMAGE_HEADER_Y + (imageAreaH - targetH) / 2;

        uint8_t* frame = display.getBuffer();
        const uint32_t sourceRowBytes = (w + 7) / 8;

        for (int dy = 0; dy < targetH; dy++) {
            const int sy = min(h - 1, (dy * h) / targetH);
            for (int dx = 0; dx < targetW; dx++) {
                const int sx = min(w - 1, (dx * w) / targetW);
                const uint8_t source = monoImgBuf[(uint32_t)sy * sourceRowBytes + (sx >> 3)];
                const bool whitePixel = (source & (0x80 >> (sx & 7))) != 0;
                frame[(targetY + dy) * DISPLAY_W + targetX + dx] = whitePixel ? 255 : 0;
            }
        }
    }

    return true;
}

// ============================================================
// UART line parser
// ============================================================

char lineBuf[360];
int lineLen = 0;

void renderPendingWithoutImage() {
    if (!pendingCardValid)
        return;

    renderUi(
        pendingCardNo,
        pendingTotal,
        pendingFolder,
        pendingText,
        false,
        0,
        0,
        0,
        nullptr,
        nullptr
    );
}

void handleCardLine(char* s) {
    const char side = s[0];
    (void)side; // Side is intentionally removed from the user-facing UI.

    if ((side != 'F' && side != 'B') || s[1] != '\t')
        return;

    char* p = s + 2;

    char* t1 = strchr(p, '\t');
    if (!t1) return;
    *t1 = '\0';

    char* t2 = strchr(t1 + 1, '\t');
    if (!t2) return;
    *t2 = '\0';

    pendingCardNo = (uint16_t)atoi(p);
    pendingTotal = (uint16_t)atoi(t1 + 1);

    /* New format: F\tcard\ttotal\tfolder\ttext */
    char* t3 = strchr(t2 + 1, '\t');

    if (t3) {
        *t3 = '\0';
        strncpy(pendingFolder, t2 + 1, MAX_FOLDER_NAME);
        pendingFolder[MAX_FOLDER_NAME] = '\0';
        strncpy(pendingText, t3 + 1, MAX_TEXT);
        pendingText[MAX_TEXT] = '\0';
    } else {
        /* Backward-compatible old format: F\tcard\ttotal\ttext */
        strncpy(pendingFolder, "OllO", MAX_FOLDER_NAME);
        pendingFolder[MAX_FOLDER_NAME] = '\0';
        strncpy(pendingText, t2 + 1, MAX_TEXT);
        pendingText[MAX_TEXT] = '\0';
    }

    pendingCardValid = true;
}

void handleLine(char* s) {
    if (!s || !s[0])
        return;

    if (s[0] == 'N') {
        pendingCardValid = true;
        renderPendingWithoutImage();
        pendingCardValid = false;
        return;
    }

    if (s[0] == 'I') {
        int w = 0;
        int h = 0;
        unsigned long len = 0;
        int format = 0;

        const int fields = sscanf(
            s,
            "I\t%d\t%d\t%lu\t%d",
            &w,
            &h,
            &len,
            &format
        );

        if (fields < 3)
            return;

        if (fields == 3)
            format = 0;

        const bool ok = receiveImage(
            (uint16_t)w,
            (uint16_t)h,
            (uint32_t)len,
            (uint8_t)format
        );

        if (!ok) {
            Serial.println("Image receive failed; showing card text only");
            renderPendingWithoutImage();
        }

        pendingCardValid = false;
        return;
    }

    handleCardLine(s);
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("================================");
    Serial.println("Ollo RP2040 display driver");
    Serial.println("LVGL 9.6 / RGB565 UI");
    Serial.println("================================");

    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.setFIFOSize(4096);
    Serial1.begin(UART_BAUD);

    if (!display.begin()) {
        Serial.println("DVI failed");
        while (true) delay(1000);
    }

    initColorPalette();
    initLvgl();

    strncpy(pendingFolder, "OllO", MAX_FOLDER_NAME);
    pendingFolder[MAX_FOLDER_NAME] = '\0';
    strncpy(pendingText, "Ollo ready", MAX_TEXT);
    pendingText[MAX_TEXT] = '\0';
    pendingCardNo = 0;
    pendingTotal = 0;
    pendingCardValid = true;
    renderPendingWithoutImage();
    pendingCardValid = false;

    Serial.println("RP2040 ready");
}

void loop() {
    while (Serial1.available()) {
        const char c = Serial1.read();

        if (c == '\r')
            continue;

        if (c == '\n') {
            lineBuf[lineLen] = '\0';
            if (lineLen > 0)
                handleLine(lineBuf);
            lineLen = 0;
        } else if (lineLen < (int)sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
        } else {
            lineLen = 0;
        }
    }

    /* LVGL reads the real clock through lv_tick_set_cb() in initLvgl(). */
    lv_timer_handler();
    delay(1);
}
