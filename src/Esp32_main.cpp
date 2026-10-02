/*
 * OllO - ESP32-S3 Super Mini
 * BLE deck sync + LittleFS storage + UART display bridge + touch pads
 *
 * BLE protocol:
 *
 * Service:
 *   6f6c6c6f-0001-4000-8000-00805f9b34fb
 *
 * Write:
 *   6f6c6c6f-0002-4000-8000-00805f9b34fb
 *
 * Notify:
 *   6f6c6c6f-0003-4000-8000-00805f9b34fb
 *
 * Packet:
 *   [type][payload]
 *
 * App -> ESP:
 *   01 BEGIN_SYNC   u16 cardCount
 *   02 CARD         u16 index, u32 frontImg, u32 backImg,
 *                   u8 frontLen, u8 backLen,
 *                   frontText, backText
 *   03 IMG_BEGIN    u32 id, u16 width, u16 height, u32 dataLen
 *   04 IMG_CHUNK    u32 offset, bytes...
 *   05 IMG_END      u8 checksum
 *   06 END_SYNC
 *
 * ESP -> App:
 *   80 refType status
 *
 * UART to RP2040:
 *   I\tw\th\tlen\n + raw image + checksum
 *   N\n
 *   F/B\tcard\ttotal\ttext\n
 *
 * RP2040 -> ESP32:
 *   NEXT\n
 *   PREV\n
 *   FLIP\n
 *
 * Touch pads (2x TTP223):
 *   pin 7: tap = FLIP, double tap = NEXT
 *   pin 6: tap = FLIP, double tap = PREV
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

#define UART_TX 4       // ESP32 GPIO4 -> RP2040 GP13 RX
#define UART_RX 5       // ESP32 GPIO5 <- RP2040 GP12 TX
#define UART_BAUD 230400

HardwareSerial RPSerial(1);

// ============================================================
// Touch pins (TTP223 OUT pins)
// ============================================================

#define TOUCH_BACK_PIN   6   // pad nearest the back of the temple arm
#define TOUCH_FRONT_PIN  7   // pad nearest the front

// ============================================================
// Limits
// ============================================================

#define MAX_TEXT 100
#define MAX_IMAGE_BYTES 10240
#define MAX_CARDS 65535

// ============================================================
// BLE
// ============================================================

NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* txCharacteristic = nullptr;

bool bleConnected = false;
volatile bool bleDropped = false;   // set by BLE task, handled in loop()

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
    snprintf(path, sizeof(path), "/i%08lX.bin",
             (unsigned long)id);
    return String(path);
}

String imageTempPath(uint32_t id) {
    char path[32];
    snprintf(path, sizeof(path), "/i%08lX.tmp",
             (unsigned long)id);
    return String(path);
}

// ============================================================
// BLE notification
// ============================================================

void sendStatus(uint8_t refType, uint8_t status) {
    if (!txCharacteristic || !bleConnected)
        return;

    uint8_t packet[3];

    packet[0] = 0x80;
    packet[1] = refType;
    packet[2] = status;

    txCharacteristic->setValue(packet, sizeof(packet));
    txCharacteristic->notify();
}

// ============================================================
// Deck file helpers
// ============================================================

void deleteOldDeck() {
    if (LittleFS.exists("/cards.txt"))
        LittleFS.remove("/cards.txt");
}

/*
 * Count the cards stored in /cards.txt so the card count is
 * correct after a reboot or an interrupted sync.
 */
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

    c.frontImg =
        strtoul(line.substring(0, p1).c_str(), nullptr, 16);

    c.backImg =
        strtoul(line.substring(p1 + 1, p2).c_str(), nullptr, 16);

    // Stored order: frontImg \t backImg \t frontText \t backText
    c.front =
        line.substring(p2 + 1, p3);

    c.back =
        line.substring(p3 + 1);

    return true;
}

bool readCard(uint16_t index, Card& card) {

    File f = LittleFS.open("/cards.txt", "r");

    if (!f)
        return false;

    uint16_t n = 0;

    while (f.available()) {

        String line = f.readStringUntil('\n');

        // Only strip '\r'. Do NOT use trim(): it would remove the
        // trailing tab of a card whose back text is empty.
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
// Image reference tracking
// ============================================================

void recordImageReference(uint32_t id) {

    if (!id || !refsFile)
        return;

    char buf[16];

    snprintf(buf, sizeof(buf), "%08lX\n",
             (unsigned long)id);

    refsFile.print(buf);
}

bool imageIsReferenced(uint32_t id) {

    if (!LittleFS.exists("/sync_refs.txt"))
        return false;

    File f = LittleFS.open("/sync_refs.txt", "r");

    if (!f)
        return false;

    char wanted[16];

    snprintf(wanted, sizeof(wanted), "%08lX",
             (unsigned long)id);

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

// ============================================================
// Remove images not referenced by new deck
// ============================================================

void cleanupUnusedImages() {

    File root = LittleFS.open("/");

    if (!root)
        return;

    File file = root.openNextFile();

    while (file) {

        String name = file.name();

        file.close();

        if (name.startsWith("/i") &&
            name.endsWith(".bin")) {

            String hex =
                name.substring(2, name.length() - 4);

            uint32_t id =
                strtoul(hex.c_str(), nullptr, 16);

            if (!imageIsReferenced(id)) {
                LittleFS.remove(name);
            }
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

    String path = imagePath(id);

    File f = LittleFS.open(path, "r");

    if (!f) {
        sendNoImage();
        return false;
    }

    if (f.size() < 4) {
        f.close();
        sendNoImage();
        return false;
    }

    uint8_t header[4];

    if (f.read(header, 4) != 4) {
        f.close();
        sendNoImage();
        return false;
    }

    uint16_t w = u16le(header);
    uint16_t h = u16le(header + 2);

    uint32_t len =
        ((uint32_t)(w + 7) / 8) * h;

    if (w == 0 ||
        h == 0 ||
        w > 320 ||
        h > 240 ||
        len > MAX_IMAGE_BYTES ||
        f.size() < (uint32_t)(4 + len)) {

        f.close();
        sendNoImage();
        return false;
    }

    RPSerial.printf(
        "I\t%u\t%u\t%lu\n",
        w,
        h,
        (unsigned long)len
    );

    uint8_t buffer[256];
    uint8_t checksum = 0;
    uint32_t left = len;

    while (left) {

        size_t wanted =
            min<uint32_t>(left, sizeof(buffer));

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

    uint32_t image =
        back ? card.backImg : card.frontImg;

    const String& text =
        back ? card.back : card.front;

    sendImageToRP(image);

    RPSerial.printf(
        "%c\t%u\t%u\t%s\n",
        back ? 'B' : 'F',
        index + 1,
        syncCardCount,
        text.c_str()
    );
}

void showCurrentCard() {

    if (syncCardCount == 0)
        return;

    sendCardToRP(currentCard, currentBack);
}

// ============================================================
// Sync abort (BLE dropped in the middle of a sync)
// ============================================================

void abortSync() {

    if (!syncActive && !receivingImage)
        return;

    if (cardsTempFile) cardsTempFile.close();
    if (refsFile)      refsFile.close();

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

    // Match the deck that is actually on flash
    countCards();

    if (currentCard >= syncCardCount)
        currentCard = 0;

    Serial.printf("abortSync: cards on flash = %u\n",
                  syncCardCount);
}

// ============================================================
// BLE callbacks
// ============================================================

class ServerCallbacks : public NimBLEServerCallbacks {

    void onConnect(
        NimBLEServer* server,
        NimBLEConnInfo& connInfo) override {

        bleConnected = true;

        Serial.println("BLE connected");
    }

    void onDisconnect(
        NimBLEServer* server,
        NimBLEConnInfo& connInfo,
        int reason) override {

        bleConnected = false;

        // File work is done later from loop(), not on the BLE task
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

        if (cardsTempFile)
            cardsTempFile.close();

        if (refsFile)
            refsFile.close();

        syncActive = false;
    }

    if (cardCount > MAX_CARDS) {
        sendStatus(0x01, 1);
        return;
    }

    LittleFS.remove("/cards.new");
    LittleFS.remove("/sync_refs.txt");

    cardsTempFile =
        LittleFS.open("/cards.new", "w");

    refsFile =
        LittleFS.open("/sync_refs.txt", "w");

    if (!cardsTempFile || !refsFile) {

        if (cardsTempFile)
            cardsTempFile.close();

        if (refsFile)
            refsFile.close();

        sendStatus(0x01, 3);
        return;
    }

    syncCardCount = cardCount;
    receivedCards = 0;
    syncActive = true;

    sendStatus(0x01, 0);

    Serial.printf(
        "BEGIN_SYNC cards=%u\n",
        cardCount
    );
}

// ============================================================
// CARD
// ============================================================

void receiveCard(const uint8_t* p, size_t len) {

    /*
     * Minimum:
     *
     * index      2
     * front img  4
     * back img   4
     * front len  1
     * back len   1
     */

    if (!syncActive || len < 12) {
        sendStatus(0x02, 1);
        return;
    }

    uint16_t index = u16le(p);
    uint32_t frontImg = u32le(p + 2);
    uint32_t backImg = u32le(p + 6);

    uint8_t frontLen = p[10];
    uint8_t backLen = p[11];

    if (frontLen > MAX_TEXT ||
        backLen > MAX_TEXT ||
        12 + frontLen + backLen != len) {

        sendStatus(0x02, 1);
        return;
    }

    if (index >= syncCardCount) {
        sendStatus(0x02, 1);
        return;
    }

    String front;
    String back;

    for (uint8_t i = 0; i < frontLen; i++)
        front += (char)p[12 + i];

    for (uint8_t i = 0; i < backLen; i++)
        back += (char)p[12 + frontLen + i];

    /*
     * Store:
     *
     * frontImgHex
     * backImgHex
     * frontText
     * backText
     */

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

    if (frontImg)
        recordImageReference(frontImg);

    if (backImg)
        recordImageReference(backImg);

    receivedCards++;

    sendStatus(0x02, 0);

    Serial.printf(
        "CARD %u/%u\n",
        receivedCards,
        syncCardCount
    );
}

// ============================================================
// IMG_BEGIN
// ============================================================

void beginImage(
    const uint8_t* p,
    size_t len) {

    if (!syncActive || len != 12) {
        sendStatus(0x03, 1);
        return;
    }

    uint32_t id = u32le(p);
    uint16_t w = u16le(p + 4);
    uint16_t h = u16le(p + 6);
    uint32_t dataLen = u32le(p + 8);

    uint32_t expected =
        ((uint32_t)(w + 7) / 8) * h;

    if (id == 0 ||
        w == 0 ||
        h == 0 ||
        w > 320 ||
        h > 240 ||
        dataLen != expected ||
        dataLen > MAX_IMAGE_BYTES) {

        sendStatus(0x03, 1);
        return;
    }

    String finalPath = imagePath(id);

    /*
     * Identical image already exists.
     *
     * The ID is defined by the app from:
     * width + height + pixel data.
     */

    if (LittleFS.exists(finalPath)) {

        sendStatus(0x03, 2);

        Serial.printf(
            "IMG %08lX already exists\n",
            (unsigned long)id
        );

        return;
    }

    /*
     * Make sure we have reasonable free space.
     */

    size_t freeBytes =
        LittleFS.totalBytes() - LittleFS.usedBytes();

    if (freeBytes < dataLen + 16) {

        sendStatus(0x03, 3);
        return;
    }

    if (receivingImage) {

        if (imageFile)
            imageFile.close();

        receivingImage = false;
    }

    LittleFS.remove(imageTempPath(id));

    imageFile =
        LittleFS.open(imageTempPath(id), "w");

    if (!imageFile) {

        sendStatus(0x03, 3);
        return;
    }

    /*
     * Store the same 4-byte header used by the old
     * ESP32 firmware:
     *
     * width u16 LE
     * height u16 LE
     */

    uint8_t header[4];

    putU16(header, w);
    putU16(header + 2, h);

    if (imageFile.write(header, 4) != 4) {

        imageFile.close();
        LittleFS.remove(imageTempPath(id));

        sendStatus(0x03, 3);
        return;
    }

    imageId = id;
    imageW = w;
    imageH = h;
    imageLen = dataLen;
    imageReceived = 0;
    imageChecksum = 0;

    receivingImage = true;

    sendStatus(0x03, 0);

    Serial.printf(
        "IMG_BEGIN id=%08lX %ux%u len=%lu\n",
        (unsigned long)id,
        w,
        h,
        (unsigned long)dataLen
    );
}

// ============================================================
// IMG_CHUNK
// ============================================================

void receiveImageChunk(
    const uint8_t* p,
    size_t len) {

    if (!receivingImage || len < 4)
        return;

    uint32_t offset = u32le(p);

    const uint8_t* data = p + 4;
    size_t dataLen = len - 4;

    /*
     * Only accept sequential writes.
     *
     * This is important because the ESP never needs
     * to seek around or buffer the whole image.
     */

    if (offset != imageReceived) {

        Serial.printf(
            "IMG offset error: got=%lu expected=%lu\n",
            (unsigned long)offset,
            (unsigned long)imageReceived
        );

        if (imageFile)
            imageFile.close();

        receivingImage = false;

        return;
    }

    if (imageReceived + dataLen > imageLen) {

        if (imageFile)
            imageFile.close();

        receivingImage = false;

        return;
    }

    if (imageFile.write(data, dataLen) != dataLen) {

        if (imageFile)
            imageFile.close();

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

void endImage(
    const uint8_t* p,
    size_t len) {

    if (!receivingImage || len != 1) {
        sendStatus(0x05, 1);
        return;
    }

    uint8_t expectedChecksum = p[0];

    bool good =
        imageReceived == imageLen &&
        imageChecksum == expectedChecksum;

    if (imageFile)
        imageFile.close();

    String tempPath = imageTempPath(imageId);
    String finalPath = imagePath(imageId);

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

    Serial.printf(
        "IMG complete %08lX\n",
        (unsigned long)imageId
    );

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

        if (cardsTempFile)
            cardsTempFile.close();

        if (refsFile)
            refsFile.close();

        syncActive = false;

        LittleFS.remove("/cards.new");
        LittleFS.remove("/sync_refs.txt");

        countCards();   // keep the count of the deck still on flash

        sendStatus(0x06, 1);

        Serial.println("END_SYNC card count mismatch");

        return;
    }

    if (cardsTempFile)
        cardsTempFile.close();

    if (refsFile)
        refsFile.close();

    /*
     * Atomically replace the old deck.
     */

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

    /*
     * Delete images no longer referenced.
     */

    cleanupUnusedImages();

    LittleFS.remove("/sync_refs.txt");

    syncActive = false;

    currentCard = 0;
    currentBack = false;

    sendStatus(0x06, 0);

    Serial.printf(
        "SYNC COMPLETE: %u cards\n",
        syncCardCount
    );

    /*
     * Show the first card immediately after sync.
     */

    delay(50);
    showCurrentCard();
}

// ============================================================
// BLE packet dispatcher
// ============================================================

void handleBLEPacket(
    const uint8_t* data,
    size_t len) {

    if (!data || len < 1)
        return;

    uint8_t type = data[0];

    const uint8_t* payload = data + 1;
    size_t payloadLen = len - 1;

    switch (type) {

        case 0x01: // BEGIN_SYNC

            if (payloadLen != 2) {
                sendStatus(0x01, 1);
                return;
            }

            beginSync(u16le(payload));
            break;

        case 0x02: // CARD

            receiveCard(payload, payloadLen);
            break;

        case 0x03: // IMG_BEGIN

            beginImage(payload, payloadLen);
            break;

        case 0x04: // IMG_CHUNK

            receiveImageChunk(payload, payloadLen);
            break;

        case 0x05: // IMG_END

            endImage(payload, payloadLen);
            break;

        case 0x06: // END_SYNC

            if (payloadLen != 0) {
                sendStatus(0x06, 1);
                return;
            }

            endSync();
            break;

        default:

            Serial.printf(
                "Unknown BLE packet: 0x%02X\n",
                type
            );

            break;
    }
}

// ============================================================
// BLE write callback
// ============================================================

class WriteCallbacks : public NimBLECharacteristicCallbacks {

    void onWrite(
        NimBLECharacteristic* characteristic,
        NimBLEConnInfo& connInfo) override {

        std::string value =
            characteristic->getValue();

        if (value.empty())
            return;

        handleBLEPacket(
            reinterpret_cast<const uint8_t*>(value.data()),
            value.size()
        );
    }
};

// ============================================================
// Card navigation commands (from RP2040 UART or touch pads)
// ============================================================

String rpLine;

void handleRPCommand(const String& command) {

    Serial.printf("cmd %s  syncActive=%d cards=%u cur=%u back=%d\n",
                  command.c_str(),
                  (int)syncActive,
                  syncCardCount,
                  currentCard,
                  (int)currentBack);

    if (syncActive)
        return;

    if (syncCardCount == 0)
        return;

    if (command == "NEXT") {

        if (currentCard + 1 < syncCardCount)
            currentCard++;

        currentBack = false;
        showCurrentCard();
    }

    else if (command == "PREV") {

        if (currentCard > 0)
            currentCard--;

        currentBack = false;
        showCurrentCard();
    }

    else if (command == "FLIP") {

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
        }

        else if (rpLine.length() < 32) {

            rpLine += c;
        }
    }
}

// ============================================================
// Touch pads (TTP223) - tap logic
//
//   pin 7 (front):  1 tap = FLIP     2 taps = NEXT card
//   pin 6 (back):   1 tap = FLIP     2 taps = PREVIOUS card
//
// Each pad counts touches on its own. The first touch starts a
// short window; a second touch inside it makes a double tap,
// otherwise the single tap fires when the window runs out.
// ============================================================

#define DOUBLE_TAP_MS      400   // max time between the two taps
#define TOUCH_COOLDOWN_MS  150   // ignore actions right after one fired

struct TapPad {
    uint8_t  pin;
    bool     last;
    uint8_t  taps;
    uint32_t lastTapAt;
};

TapPad padBack  = { TOUCH_BACK_PIN,  false, 0, 0 };   // pin 6
TapPad padFront = { TOUCH_FRONT_PIN, false, 0, 0 };   // pin 7

enum PadResult { PAD_NONE, PAD_SINGLE, PAD_DOUBLE };

uint32_t touchCooldownUntil = 0;

PadResult pollPad(TapPad& p, uint32_t now) {

    bool down = digitalRead(p.pin);
    bool rise = down && !p.last;

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

    uint32_t now = millis();

    PadResult f = pollPad(padFront, now);   // pin 7
    PadResult b = pollPad(padBack,  now);   // pin 6

    if ((int32_t)(now - touchCooldownUntil) < 0)
        return;

    if (f == PAD_DOUBLE) {
        Serial.println("pin7 double -> NEXT");
        handleRPCommand("NEXT");
    }
    else if (f == PAD_SINGLE) {
        Serial.println("pin7 single -> FLIP");
        handleRPCommand("FLIP");
    }
    else if (b == PAD_DOUBLE) {
        Serial.println("pin6 double -> PREV");
        handleRPCommand("PREV");
    }
    else if (b == PAD_SINGLE) {
        Serial.println("pin6 single -> FLIP");
        handleRPCommand("FLIP");
    }
    else {
        return;
    }

    touchCooldownUntil = now + TOUCH_COOLDOWN_MS;
}

// ============================================================
// Setup
// ============================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("================================");
    Serial.println("Ollo ESP32-S3");
    Serial.println("BLE deck manager");
    Serial.println("================================");

    // -----------------------------
    // LittleFS
    // -----------------------------

    if (!LittleFS.begin(true)) {

        Serial.println("LittleFS mount FAILED");

        while (true)
            delay(1000);
    }

    Serial.printf(
        "LittleFS: %u / %u bytes used\n",
        (unsigned)LittleFS.usedBytes(),
        (unsigned)LittleFS.totalBytes()
    );

    // Restore the saved deck so navigation works after a reboot
    countCards();

    Serial.printf("Cards on flash: %u\n", syncCardCount);

    // -----------------------------
    // UART
    // -----------------------------

    RPSerial.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX,
        UART_TX
    );

    // -----------------------------
    // Touch pads
    // -----------------------------

    pinMode(TOUCH_BACK_PIN,  INPUT);
    pinMode(TOUCH_FRONT_PIN, INPUT);

    padBack.last  = digitalRead(TOUCH_BACK_PIN);
    padFront.last = digitalRead(TOUCH_FRONT_PIN);

    // -----------------------------
    // BLE
    // -----------------------------

    NimBLEDevice::init("Ollo");

    bleServer =
        NimBLEDevice::createServer();

    bleServer->setCallbacks(
        new ServerCallbacks()
    );

    NimBLEService* service =
        bleServer->createService(SERVICE_UUID);

    txCharacteristic =
        service->createCharacteristic(
            NOTIFY_UUID,
            NIMBLE_PROPERTY::NOTIFY
        );

    NimBLECharacteristic* rxCharacteristic =
        service->createCharacteristic(
            WRITE_UUID,
            NIMBLE_PROPERTY::WRITE_NR
        );

    rxCharacteristic->setCallbacks(
        new WriteCallbacks()
    );

    service->start();

    NimBLEAdvertising* advertising =
        NimBLEDevice::getAdvertising();

    advertising->addServiceUUID(
        SERVICE_UUID
    );

    advertising->setName("Ollo");

    advertising->start();

    Serial.println("BLE advertising as Ollo");

    // Show the saved deck on the display after boot
    delay(500);   // give the RP2040 time to start
    showCurrentCard();
}

// ============================================================
// Loop
// ============================================================

void loop() {

    if (bleDropped) {
        bleDropped = false;
        abortSync();
    }

    readRPCommands();
    readTouch();

    delay(1);
}
