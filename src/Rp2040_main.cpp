/*
 * OllO - RP2040 display driver
 *
 * Arduino-Pico + PicoDVI
 *
 * ESP32 -> RP2040:
 *
 * Image:
 *   I\tw\th\len\n
 *   + len raw bytes
 *   + 1 checksum byte
 *
 * No image:
 *   N\n
 *
 * Text:
 *   F/B\tcardNo\ttotal\ttext\n
 *
 * RP2040 -> ESP32:
 *   NEXT\n
 *   PREV\n
 *   FLIP\n
 */

#include <Arduino.h>
#include <PicoDVI.h>

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

DVIGFX1 display(
    DVI_RES_640x480p60,
    false,
    my_dvi_cfg
);

// ============================================================
// UART
// ============================================================

#define UART_TX_PIN 12
#define UART_RX_PIN 13
#define UART_BAUD 230400

// ESP GPIO4 -> RP GP13
// ESP GPIO5 <- RP GP12

// ============================================================
// Display settings
// ============================================================

#define VISIBLE_H 480

#define COL_BG 1
#define COL_FG 0

const int MARGIN = 16;
const int HEADER_H = 32;

#define MAX_LINE 320
#define MAX_ROWS 40

// ============================================================
// Image buffer
// ============================================================

#define IMG_MAX_W 320
#define IMG_MAX_H 240

static uint8_t imgBuf[
    (IMG_MAX_W / 8) * IMG_MAX_H
];

bool hasImg = false;

int imgW = 0;
int imgH = 0;

// ============================================================
// UART line buffer
// ============================================================

char lineBuf[MAX_LINE];
int lineLen = 0;

// ============================================================
// Text layout
// ============================================================

int layout(
    const char* s,
    int cols,
    const char* ls[],
    int ll[]
) {

    int n = 0;

    const char* p = s;

    while (*p && n < MAX_ROWS) {

        while (*p == ' ')
            p++;

        if (!*p)
            break;

        if (p[0] == '\\' && p[1] == 'n') {

            ls[n] = p;
            ll[n] = 0;

            n++;

            p += 2;

            continue;
        }

        int len = 0;
        int lastSpace = -1;

        while (
            p[len] &&
            len < cols &&
            !(p[len] == '\\' &&
              p[len + 1] == 'n')
        ) {

            if (p[len] == ' ')
                lastSpace = len;

            len++;
        }

        int take;

        if (
            !p[len] ||
            p[len] == ' ' ||
            (p[len] == '\\' &&
             p[len + 1] == 'n')
        ) {

            take = len;

        } else if (lastSpace > 0) {

            take = lastSpace;

        } else {

            take = len;
        }

        int t = take;

        while (
            t > 0 &&
            p[t - 1] == ' '
        ) {

            t--;
        }

        ls[n] = p;
        ll[n] = t;

        n++;

        p += take;

        if (
            p[0] == '\\' &&
            p[1] == 'n'
        ) {

            p += 2;
        }
    }

    return n;
}

// ============================================================
// Render
// ============================================================

void render(
    char side,
    int idx,
    int total,
    const char* text
) {

    const int W = display.width();

    const int H =
        min(
            (int)display.height(),
            (int)VISIBLE_H
        );

    display.fillScreen(COL_BG);

    // -----------------------------
    // Header
    // -----------------------------

    display.setTextSize(2);

    int labelW;

    if (side == 'B') {

        display.setTextColor(
            COL_BG,
            COL_FG
        );

        display.setCursor(
            MARGIN,
            MARGIN
        );

        display.print(" BACK ");

        labelW = 6 * 12;

    } else {

        display.setTextColor(
            COL_FG,
            COL_BG
        );

        display.setCursor(
            MARGIN,
            MARGIN
        );

        display.print("FRONT");

        labelW = 5 * 12;
    }

    // -----------------------------
    // Progress bar
    // -----------------------------

    if (total > 0) {

        int barX =
            MARGIN +
            labelW +
            16;

        int barW =
            W -
            MARGIN -
            barX;

        int barY =
            MARGIN + 1;

        display.drawRect(
            barX,
            barY,
            barW,
            14,
            COL_FG
        );

        int fillW =
            (long)(barW - 4) *
            idx /
            total;

        display.fillRect(
            barX + 2,
            barY + 2,
            fillW,
            10,
            COL_FG
        );
    }

    display.drawFastHLine(
        MARGIN,
        MARGIN + 24,
        W - 2 * MARGIN,
        COL_FG
    );

    // -----------------------------
    // Body
    // -----------------------------

    const int areaTop =
        MARGIN + HEADER_H;

    int bodyTop = areaTop;

    int maxSize = 5;

    if (hasImg) {

        display.drawBitmap(
            (W - imgW) / 2,
            areaTop + 4,
            imgBuf,
            imgW,
            imgH,
            COL_FG,
            COL_BG
        );

        bodyTop =
            areaTop +
            imgH +
            12;

        maxSize = 3;
    }

    const int areaH =
        H -
        bodyTop -
        MARGIN;

    const char* ls[MAX_ROWS];
    int ll[MAX_ROWS];

    int size = maxSize;
    int n = 0;

    for (; size >= 1; size--) {

        int cols =
            (W - 2 * MARGIN) /
            (6 * size);

        n =
            layout(
                text,
                cols,
                ls,
                ll
            );

        if (
            n *
            (8 * size + size * 2)
            <= areaH
        ) {

            break;
        }
    }

    if (size < 1)
        size = 1;

    const int cw =
        6 * size;

    const int lh =
        8 * size +
        size * 2;

    int y0 =
        bodyTop +
        (areaH - n * lh) / 2;

    if (y0 < bodyTop)
        y0 = bodyTop;

    for (int i = 0; i < n; i++) {

        int x0 =
            (W - ll[i] * cw) / 2;

        for (int j = 0; j < ll[i]; j++) {

            display.drawChar(
                x0 + j * cw,
                y0 + i * lh,
                ls[i][j],
                COL_FG,
                COL_BG,
                size
            );
        }
    }
}

// ============================================================
// Receive image
// ============================================================

void receiveImage(
    int w,
    int h,
    uint32_t len
) {

    hasImg = false;

    bool ok =
        w > 0 &&
        h > 0 &&
        w <= IMG_MAX_W &&
        h <= IMG_MAX_H &&
        len ==
            (uint32_t)((w + 7) / 8) * h;

    uint32_t got = 0;

    uint8_t sum = 0;
    uint8_t chk = 0;

    uint32_t start = millis();

    while (
        got < len + 1 &&
        millis() - start < 3000
    ) {

        if (Serial1.available()) {

            uint8_t b =
                Serial1.read();

            start = millis();

            if (got < len) {

                if (ok)
                    imgBuf[got] = b;

                sum += b;

            } else {

                chk = b;
            }

            got++;
        }
    }

    if (
        ok &&
        got == len + 1 &&
        sum == chk
    ) {

        imgW = w;
        imgH = h;
        hasImg = true;
    }
}

// ============================================================
// Handle incoming ESP line
// ============================================================

void handleLine(char* s) {

    // -----------------------------
    // No image
    // -----------------------------

    if (s[0] == 'N') {

        hasImg = false;

        return;
    }

    // -----------------------------
    // Image header
    // -----------------------------

    if (s[0] == 'I') {

        int w;
        int h;

        unsigned len;

        if (
            sscanf(
                s,
                "I\t%d\t%d\t%u",
                &w,
                &h,
                &len
            ) == 3
        ) {

            receiveImage(
                w,
                h,
                len
            );
        }

        return;
    }

    // -----------------------------
    // Card text
    // -----------------------------

    char side = s[0];

    if (
        (side != 'F' &&
         side != 'B') ||
        s[1] != '\t'
    ) {

        return;
    }

    char* p = s + 2;

    char* t1 =
        strchr(p, '\t');

    if (!t1)
        return;

    *t1 = 0;

    int idx =
        atoi(p);

    p = t1 + 1;

    char* t2 =
        strchr(p, '\t');

    if (!t2)
        return;

    *t2 = 0;

    int total =
        atoi(p);

    render(
        side,
        idx,
        total,
        t2 + 1
    );
}

// ============================================================
// Setup
// ============================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println(
        "Ollo RP2040 starting..."
    );

    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);

    Serial1.setFIFOSize(4096);

    Serial1.begin(UART_BAUD);

    if (!display.begin()) {

        Serial.println(
            "DVI failed"
        );

        while (true)
            delay(1000);
    }

    render(
        'F',
        0,
        0,
        "Ollo ready"
    );

    Serial.println(
        "RP2040 ready"
    );
}

// ============================================================
// Loop
// ============================================================

void loop() {

    while (Serial1.available()) {

        char c =
            Serial1.read();

        if (c == '\r')
            continue;

        if (c == '\n') {

            lineBuf[lineLen] = 0;

            if (lineLen > 0)
                handleLine(lineBuf);

            lineLen = 0;

        } else {

            if (
                lineLen <
                MAX_LINE - 1
            ) {

                lineBuf[lineLen++] = c;
            }
        }
    }
}