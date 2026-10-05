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
#define UART_BAUD 230400

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

bool bleConnected = false;
volatile bool bleDropped = false;

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
// UART display bridge
// ============================================================

void sendNoImage() {
    RPSerial.print("N\n");
}

bool sendImageToRP(uint32_t id) {
    if (id == 0) {
        sendNoImage();
        return true;
    }

    File f = LittleFS.open(imagePath(id), "r");
    if (!f) {
        sendNoImage();
        return false;
    }

    const uint32_t fileSize = (uint32_t)f.size();
    if (fileSize < 4) {
        f.close();
        sendNoImage();
        return false;
    }

    uint8_t wh[4];
    if (f.read(wh, 4) != 4) {
        f.close();
        sendNoImage();
        return false;
    }

    const uint16_t w = u16le(wh);
    const uint16_t h = u16le(wh + 2);

    const uint32_t monoLen = ((uint32_t)(w + 7) / 8) * h;
    const uint32_t colorLen = (uint32_t)w * h * 2UL;
    const uint32_t remaining = fileSize - 4;

    uint8_t format = IMAGE_FORMAT_MONO;
    bool hasFormatHeader = false;
    uint32_t len = 0;

    if (remaining == colorLen + 1 && w > 0 && h > 0 && w <= MAX_COLOR_IMG_W && h <= MAX_COLOR_IMG_H) {
        format = IMAGE_FORMAT_RGB565;
        hasFormatHeader = true;
        len = colorLen;
    } else if (remaining == monoLen + 1 && w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H) {
        format = IMAGE_FORMAT_MONO;
        hasFormatHeader = true;
        len = monoLen;
    } else if (remaining == monoLen && w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H) {
        // Backward-compatible old mono image header.
        format = IMAGE_FORMAT_MONO;
        hasFormatHeader = false;
        len = monoLen;
    } else {
        f.close();
        sendNoImage();
        return false;
    }

    if (hasFormatHeader) {
        uint8_t storedFormat = 0;
        if (f.read(&storedFormat, 1) != 1 || storedFormat != format) {
            f.close();
            sendNoImage();
            return false;
        }
    }

    RPSerial.printf(
        "I\t%u\t%u\t%lu\t%u\n",
        w,
        h,
        (unsigned long)len,
        (unsigned)format
    );

    uint8_t buffer[256];
    uint8_t checksum = 0;
    uint32_t left = len;

    while (left) {
        size_t wanted = min<uint32_t>(left, sizeof(buffer));
        size_t n = f.read(buffer, wanted);
        if (n == 0) {
            f.close();
            return false;
        }

        for (size_t i = 0; i < n; i++)
            checksum += buffer[i];

        RPSerial.write(buffer, n);
        left -= n;
    }

    RPSerial.write(checksum);
    f.close();
    return true;
}

void sendCardToRP(uint16_t index, bool back) {
    Card card;
    if (!readCard(index, card))
        return;

    const uint32_t image = back ? card.backImg : card.frontImg;
    const String& text = back ? card.back : card.front;

    // Send metadata first. The RP2040 prepares its UI before receiving a color image.
    RPSerial.printf(
        "%c\t%u\t%u\t%s\t%s\n",
        back ? 'B' : 'F',
        index + 1,
        syncCardCount,
        card.folder.c_str(),
        text.c_str()
    );

    sendImageToRP(image);
}

void showCurrentCard() {
    if (syncCardCount == 0)
        return;

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
        if (value.empty())
            return;

        handleBLEPacket(
            reinterpret_cast<const uint8_t*>(value.data()),
            value.size()
        );
    }
};

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
        abortSync();
    }

    readRPCommands();
    readTouch();
    delay(1);
}
    