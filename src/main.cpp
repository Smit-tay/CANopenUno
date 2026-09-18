#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

const uint8_t NODE_ID = 0x20;

enum NmtState { INITIALISING = 0, STOPPED = 4, OPERATIONAL = 5, PRE_OPERATIONAL = 127 };
NmtState currentState = INITIALISING;

MCP_CAN CAN0(10);

/* ============================== Object Dictionary =========================
 * Each entry is one backing variable, its size, and its access rights.
 * Adding a new SDO-visible object is one line here - nothing else in the
 * SDO layer below needs to know it exists. */

uint32_t od_device_type = 0x00000000;      // 0x1000, RO, 4 bytes
uint8_t  od_error_reg = 0x00;              // 0x1001, RO, 1 byte
uint16_t od_heartbeat_ms = 1000;           // 0x1017, RW, 2 bytes
uint16_t od_sim_sensor = 0x01A0;           // 0x2100, RW, 2 bytes - published via TPDO1
uint8_t  od_test_buffer[10] = { 0 };       // 0x2101, RW, 10 bytes - forces segmented transfer

const uint8_t LED_PIN = 7;
bool od_led_state = false;

enum OdAccess : uint8_t { OD_RO = 1, OD_WO = 2, OD_RW = 3 };

struct OdEntry {
    uint16_t index;
    uint8_t  subIndex;
    OdAccess access;
    uint8_t* data;
    uint8_t  size;
};

const OdEntry odTable[] = {
    { 0x1000, 0, OD_RO, (uint8_t*)&od_device_type, sizeof(od_device_type) },
    { 0x1001, 0, OD_RO, (uint8_t*)&od_error_reg,   sizeof(od_error_reg)   },
    { 0x1017, 0, OD_RW, (uint8_t*)&od_heartbeat_ms, sizeof(od_heartbeat_ms) },
    { 0x2100, 0, OD_RW, (uint8_t*)&od_sim_sensor,   sizeof(od_sim_sensor)   },
    { 0x2101, 0, OD_RW, od_test_buffer,             sizeof(od_test_buffer)  },
    { 0x2102, 0, OD_RW, (uint8_t*)&od_led_state,    sizeof(od_led_state) },
};
const uint8_t ODTABLE_COUNT = sizeof(odTable) / sizeof(odTable[0]);

const OdEntry* odFind(uint16_t index, uint8_t subIndex) {
    for (uint8_t i = 0; i < ODTABLE_COUNT; i++) {
        if (odTable[i].index == index && odTable[i].subIndex == subIndex) {
            return &odTable[i];
        }
    }
    return nullptr;
}

/* ==================================== SDO ==================================
 * Abort codes match CO_SDO_abortCode_t exactly (see CANopenNode's own
 * CO_SDOserver.h) - only the handful this implementation actually uses.
 * All cs-byte formats below are taken from CANopenNode's real CO_SDOserver.c
 * (segment count = 7-((cs>>1)&0x07), download-segment-ack = 0x20|toggle,
 * upload-initiate-response = 0x43|((4-size)<<2) expedited / 0x41 segmented
 * with size in bytes 4-7 / 0x40 if size unknown), not derived from memory. */

const uint32_t SDO_AB_TOGGLE_BIT = 0x05030000UL;
const uint32_t SDO_AB_CMD = 0x05040001UL;
const uint32_t SDO_AB_WRITEONLY = 0x06010001UL;
const uint32_t SDO_AB_READONLY = 0x06010002UL;
const uint32_t SDO_AB_NOT_EXIST = 0x06020000UL;
const uint32_t SDO_AB_DATA_LONG = 0x06070012UL;

enum SdoXferDir : uint8_t { SDO_XFER_NONE, SDO_XFER_DOWNLOAD, SDO_XFER_UPLOAD };

SdoXferDir sdoXferDir = SDO_XFER_NONE;
const OdEntry* sdoXferEntry = nullptr;
uint8_t sdoXferToggle = 0;
uint8_t sdoXferOffset = 0;

void logCanFrame(const char* prefix, uint32_t canId, uint8_t len, const uint8_t* buf); // fwd - defined below

void sdoSendFrame(const uint8_t* buf) {
    uint32_t txId = 0x580 + NODE_ID;
    if (CAN0.sendMsgBuf(txId, 0, 8, (uint8_t*)buf) == CAN_OK) {
        logCanFrame("TX", txId, 8, buf);
    }
}

/* Bare cs byte, everything else zero - used for segment acks, which carry
 * no index/subindex at all (that only exists in the initiate frame). */
void sdoSendCsOnly(uint8_t cs0) {
    uint8_t buf[8] = { 0 };
    buf[0] = cs0;
    sdoSendFrame(buf);
}

void sdoAbort(uint16_t index, uint8_t subIndex, uint32_t abortCode) {
    uint8_t buf[8] = { 0 };
    buf[0] = 0x80;
    buf[1] = (uint8_t)index;
    buf[2] = (uint8_t)(index >> 8);
    buf[3] = subIndex;
    memcpy(&buf[4], &abortCode, 4);
    sdoSendFrame(buf);
}

void sdoAckPlain(uint8_t cs0, uint16_t index, uint8_t subIndex) {
    uint8_t buf[8] = { 0 };
    buf[0] = cs0;
    buf[1] = (uint8_t)index;
    buf[2] = (uint8_t)(index >> 8);
    buf[3] = subIndex;
    sdoSendFrame(buf);
}

void sdoStartUpload(uint16_t index, uint8_t subIndex) {
    const OdEntry* entry = odFind(index, subIndex);
    if (entry == nullptr) {
        sdoAbort(index, subIndex, SDO_AB_NOT_EXIST);
        return;
    }
    if (entry->access == OD_WO) {
        sdoAbort(index, subIndex, SDO_AB_WRITEONLY);
        return;
    }

    uint8_t buf[8] = { 0 };
    buf[1] = (uint8_t)index;
    buf[2] = (uint8_t)(index >> 8);
    buf[3] = subIndex;

    if (entry->size <= 4) {
        buf[0] = 0x43 | ((4 - entry->size) << 2);
        memcpy(&buf[4], entry->data, entry->size);
        sdoSendFrame(buf);
        return;
    }

    buf[0] = 0x41; // segmented, size indicated
    uint32_t size32 = entry->size;
    memcpy(&buf[4], &size32, 4);
    sdoSendFrame(buf);

    sdoXferDir = SDO_XFER_UPLOAD;
    sdoXferEntry = entry;
    sdoXferToggle = 0x00;
    sdoXferOffset = 0;
}

/* Upload segment request from the client is just the toggle bit - the
 * client doesn't know how much data remains, only the server does. */
void sdoContinueUpload(uint8_t cs) {
    uint8_t toggle = cs & 0x10;
    if (toggle != sdoXferToggle) {
        sdoAbort(sdoXferEntry->index, sdoXferEntry->subIndex, SDO_AB_TOGGLE_BIT);
        sdoXferDir = SDO_XFER_NONE;
        return;
    }

    uint8_t remaining = sdoXferEntry->size - sdoXferOffset;
    uint8_t count = (remaining < 7) ? remaining : 7;
    bool isLast = (remaining <= 7);

    uint8_t buf[8] = { 0 };
    buf[0] = sdoXferToggle | (isLast ? (((7 - count) << 1) | 0x01) : 0);
    memcpy(&buf[1], sdoXferEntry->data + sdoXferOffset, count);
    sdoSendFrame(buf);

    sdoXferOffset += count;
    sdoXferToggle = (sdoXferToggle == 0x00) ? 0x10 : 0x00;
    if (isLast) {
        sdoXferDir = SDO_XFER_NONE;
    }
}

void sdoStartDownload(uint16_t index, uint8_t subIndex, uint8_t cs, const uint8_t* rxData) {
    const OdEntry* entry = odFind(index, subIndex);
    if (entry == nullptr) {
        sdoAbort(index, subIndex, SDO_AB_NOT_EXIST);
        return;
    }
    if (entry->access == OD_RO) {
        sdoAbort(index, subIndex, SDO_AB_READONLY);
        return;
    }

    bool expedited = (cs & 0x02) != 0;
    if (expedited) {
        uint8_t n = ((cs & 0x01) != 0) ? ((cs >> 2) & 0x03) : 0;
        uint8_t dataSize = 4 - n;
        if (dataSize > entry->size) {
            sdoAbort(index, subIndex, SDO_AB_DATA_LONG);
            return;
        }
        memcpy(entry->data, rxData, dataSize);
        sdoAckPlain(0x60, index, subIndex);
        return;
    }

    // Segmented initiate - no data in this frame, just start the transfer.
    sdoAckPlain(0x60, index, subIndex);
    sdoXferDir = SDO_XFER_DOWNLOAD;
    sdoXferEntry = entry;
    sdoXferToggle = 0x00;
    sdoXferOffset = 0;
}

void sdoContinueDownload(uint8_t cs, const uint8_t* rxData) {
    if ((cs & 0xE0) != 0x00) {
        sdoAbort(sdoXferEntry->index, sdoXferEntry->subIndex, SDO_AB_CMD);
        sdoXferDir = SDO_XFER_NONE;
        return;
    }
    uint8_t toggle = cs & 0x10;
    if (toggle != sdoXferToggle) {
        sdoAbort(sdoXferEntry->index, sdoXferEntry->subIndex, SDO_AB_TOGGLE_BIT);
        sdoXferDir = SDO_XFER_NONE;
        return;
    }

    uint8_t count = 7 - ((cs >> 1) & 0x07);
    bool isLast = (cs & 0x01) != 0;

    if ((uint16_t)sdoXferOffset + count > sdoXferEntry->size) {
        sdoAbort(sdoXferEntry->index, sdoXferEntry->subIndex, SDO_AB_DATA_LONG);
        sdoXferDir = SDO_XFER_NONE;
        return;
    }
    memcpy(sdoXferEntry->data + sdoXferOffset, rxData, count);
    sdoXferOffset += count;

    sdoSendCsOnly(0x20 | sdoXferToggle);
    sdoXferToggle = (sdoXferToggle == 0x00) ? 0x10 : 0x00;

    if (isLast) {
        sdoXferDir = SDO_XFER_NONE;
        Serial.print(F("Segmented download complete: "));
        Serial.print(sdoXferOffset);
        Serial.println(F(" bytes"));
    }
}

/* ============================== Message-class dispatch =====================
 * Classifies every incoming frame into the SAME category taxonomy used
 * project-wide: canopen_driver's co_driver_trace.h CO_TraceFilter_t and
 * console_ui.h's TRACE command (see today's work) - one shared set of
 * class names (Nmt/Sync/Emcy/Time/Pdo/Sdo/Hb/Other), not three
 * independently-maintained copies of the same COB-ID ranges drifting
 * apart over time.
 *
 * loop() classifies once, dispatches once, to one small handler per
 * class - adding real behavior for a class (SYNC-gated PDO, EMCY
 * production, whatever comes next) means filling in that one handler,
 * not growing an if/else chain keyed on raw COB-IDs. */

enum class MsgClass : uint8_t { Nmt, Sync, Emcy, Time, Pdo, Sdo, Hb, Other };

/* CiA301 Predefined Connection Set, function code = top 4 bits of the
 * 11-bit COB-ID - same classification canopen_driver's co_driver_trace.c
 * uses, node-id (low 7 bits) is irrelevant for classification. */
MsgClass classify(uint32_t canId) {
	
	/* NMT, SYNC, and TIME each have ONE fixed COB-ID, no node-id component -
	 * unlike PDO/SDO/heartbeat below, there's no formula to break. An exact
	 * match here is always correct, whatever COB-IDs any PDO gets reassigned
	 * to. Must run before the functionCode mask below: 0x080 also matches
	 * the EMCY function-code range, so SYNC needs to be caught here first. */
    if (canId == 0x000) return MsgClass::Nmt;
    if (canId == 0x080) return MsgClass::Sync;
    if (canId == 0x100) return MsgClass::Time;

	/* Isolate the 4-bit function code (COB-ID bits 10-7), masking off the
	 * low 7 node-id bits without shifting - result stays comparable to the
	 * predefined-connection-set base values (0x180, 0x580, 0x700, etc.)
	 * checked below:
	 *   canId  : F F F F N N N N N N N   (F = function code, N = node-id)
	 *   0x780  : 1 1 1 1 0 0 0 0 0 0 0
	 *
	 * CAVEAT: only correct for COB-IDs still on the formula
	 * (base + nodeId). A hand-assigned COB-ID outside that formula (e.g.
	 * this project's own TPDO5 at 0x690) won't match any base below and
	 * falls through to MsgClass::Other. Even on a match, canId & 0x07F is
	 * NOT reliably the sender's real node-id - only true under the same
	 * formula assumption. */
    uint16_t functionCode = canId & 0x780;

    if (functionCode == 0x080) return MsgClass::Emcy; /* 0x081-0x0FF, node!=0 */
    if (functionCode == 0x180 || functionCode == 0x200 || functionCode == 0x280 || functionCode == 0x300
        || functionCode == 0x380 || functionCode == 0x400 || functionCode == 0x480 || functionCode == 0x500) {
        return MsgClass::Pdo;
    }
    if (functionCode == 0x580 || functionCode == 0x600) return MsgClass::Sdo;
    if (functionCode == 0x700) return MsgClass::Hb;

    return MsgClass::Other;
}

void handleNmtTransition(uint8_t command); // fwd - defined below
void sendPdo1Frame(); // fwd - defined below

/* NMT target is in the payload (byte 1), not the COB-ID (always 0x000) -
 * 0x00 means "all nodes". */
void handleNmtMessage(uint8_t len, const uint8_t* rxBuf) {
    if (len == 2 && (rxBuf[1] == 0x00 || rxBuf[1] == NODE_ID)) {
        handleNmtTransition(rxBuf[0]);
    }
}

/* Top-level SDO dispatcher - a segment continuation is routed purely by
 * which direction is currently active (set by sdoStartUpload()/
 * sdoStartDownload()), since segment frames carry no index/subindex to
 * key off of at all. */
/* SDO's own COB-ID encodes the target node (0x600+NODE_ID is unique per
 * node) - unlike NMT, which uses one shared COB-ID with the target
 * selected via payload data. classify() groups ALL nodes' SDO traffic
 * under one MsgClass (function-code 0x600 matches any node's requests,
 * not just this one's) - correct for trace/display purposes, but NOT
 * sufficient for dispatch on its own. A real CANopenNode stack never
 * sees a non-matching frame at all (CAN hardware/software filtering
 * discards it before application code runs); this hand-rolled dispatcher
 * receives every frame on the bus and has to check for itself - this
 * check was missing, confirmed as the real cause of a genuine
 * cross-talk bug: with two SDO transfers to two different nodes
 * in flight at once (only possible once canopen_driver gained real
 * multi-channel SDO), this node was answering requests addressed to
 * the OTHER node too, since nothing here ever verified the target. */
void handleSdoMessage(uint32_t rxId, uint8_t len, uint8_t* rxBuf) {
    if (rxId != (0x600 + NODE_ID)) {
        return;
    }
    if (len != 8) {
        return; /* every real SDO frame on the wire is a full 8 bytes */
    }
    uint8_t cs = rxBuf[0];

    if (sdoXferDir == SDO_XFER_DOWNLOAD) {
        sdoContinueDownload(cs, &rxBuf[1]);
        return;
    }
    if (sdoXferDir == SDO_XFER_UPLOAD) {
        sdoContinueUpload(cs);
        return;
    }

    uint16_t index = (rxBuf[2] << 8) | rxBuf[1];
    uint8_t subIndex = rxBuf[3];

    if (cs == 0x40) {
        sdoStartUpload(index, subIndex);
    } else if ((cs & 0xE0) == 0x20) {
        sdoStartDownload(index, subIndex, cs, &rxBuf[4]);
    } else {
        sdoAbort(index, subIndex, SDO_AB_CMD);
    }
}

/* Stubs - no behavior yet for these classes on RX. Kept as real,
 * separate functions (not folded into a shared no-op) specifically so
 * each is an obvious, named place to add real behavior later - e.g.
 * handleSyncMessage() is where SYNC-gated TPDO transmission will go,
 * once that's built as its own separate step (see today's discussion -
 * deliberately not bundled into this structural rewrite). */
void handleSyncMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
    if (currentState == OPERATIONAL) {
        sendPdo1Frame();
    }
}

void handleEmcyMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
}

void handleTimeMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
}

void handlePdoMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
}

void handleHbMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
}

void handleOtherMessage(uint8_t len, const uint8_t* rxBuf) {
    (void)len;
    (void)rxBuf;
}

/* ============================== NMT / heartbeat / PDO ====================== */

void sendHeartbeatFrame(uint8_t stateValue) {
    uint8_t txBuf[1] = { stateValue };
    uint32_t txId = 0x700 + NODE_ID;
    if (CAN0.sendMsgBuf(txId, 0, 1, txBuf) == CAN_OK) {
        logCanFrame("TX", txId, 1, txBuf);
    }
}

void handleNmtTransition(uint8_t command) {
    switch (command) {
        case 0x01: currentState = OPERATIONAL; break;
        case 0x02: currentState = STOPPED; break;
        case 0x80: currentState = PRE_OPERATIONAL; break;
        case 0x81:
        case 0x82:
            currentState = INITIALISING;
            setup();
            break;
    }
}

void sendPdo1Frame() {
    uint8_t txData[2];
    txData[0] = od_sim_sensor & 0xFF;
    txData[1] = (od_sim_sensor >> 8) & 0xFF;
    uint32_t txId = 0x180 + NODE_ID;
    if (CAN0.sendMsgBuf(txId, 0, 2, txData) == CAN_OK) {
        logCanFrame("TX", txId, 2, txData);
    }
}

/* ============================== CAN trace logging =========================
 * Unchanged from before - same design as canopen_driver's co_driver_trace.c.
 * frameTypeName() is more granular than classify() above (TPDO1 vs TPDO2
 * etc, distinguishing PDO sub-types classify() lumps into one MsgClass::Pdo)
 * - kept separate on purpose, for display, not dispatch. */

static const char* frameTypeName(uint32_t identifier, uint8_t dlc, const uint8_t* data, uint8_t* outNode, bool* outNodeIsAll) {
    *outNode = 0;
    *outNodeIsAll = false;

    if (identifier == 0x000) {
        uint8_t target = (dlc >= 2 && data != NULL) ? data[1] : 0;
        *outNode = target;
        *outNodeIsAll = (target == 0);
        return "NMT";
    }
    if (identifier == 0x080) { *outNodeIsAll = true; return "SYNC"; }
    if (identifier == 0x100) { *outNodeIsAll = true; return "TIME"; }

    uint16_t functionCode = identifier & 0x780;
    *outNode = (uint8_t)(identifier & 0x7F);

    if (functionCode == 0x080) return "EMCY";
    if (functionCode == 0x180) return "TPDO1";
    if (functionCode == 0x200) return "RPDO1";
    if (functionCode == 0x280) return "TPDO2";
    if (functionCode == 0x300) return "RPDO2";
    if (functionCode == 0x380) return "TPDO3";
    if (functionCode == 0x400) return "RPDO3";
    if (functionCode == 0x480) return "TPDO4";
    if (functionCode == 0x500) return "RPDO4";
    if (functionCode == 0x580) return "SDOtx";
    if (functionCode == 0x600) return "SDOrx";
    if (functionCode == 0x700) return "HB";

    return "OTHR";
}

void logCanFrame(const char* prefix, uint32_t canId, uint8_t len, const uint8_t* buf) {
    static unsigned long lastTxMillis = 0;
    static unsigned long lastRxMillis = 0;
    bool isTx = (prefix[0] == 'T');

    unsigned long now = millis();
    unsigned long* lastMillis = isTx ? &lastTxMillis : &lastRxMillis;
    unsigned long delta = (*lastMillis == 0) ? 0 : (now - *lastMillis);
    *lastMillis = now;

    Serial.print(prefix);
    Serial.print(": t");
    if (canId < 0x100) Serial.print("0");
    if (canId < 0x010) Serial.print("0");
    Serial.print(canId, HEX);
    Serial.print(len, HEX);
    for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
    }

    uint8_t node = 0;
    bool nodeIsAll = false;
    const char* typeName = frameTypeName(canId, len, buf, &node, &nodeIsAll);
    Serial.print("  ");
    Serial.print(typeName);
    Serial.print(" node=");
    if (nodeIsAll) Serial.print("ALL"); else Serial.print(node);

    Serial.print("  [t=");
    Serial.print(now);
    Serial.print(" ms, +");
    Serial.print(delta);
    Serial.println(" ms]");
}

/* ==================================== setup/loop ============================ */

void setup() {
    Serial.begin(115200);
    while (!Serial);

    sdoXferDir = SDO_XFER_NONE; // in case setup() is re-entered via an NMT reset mid-transfer

    if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
        currentState = PRE_OPERATIONAL;
    }

    pinMode(LED_PIN, OUTPUT);

    CAN0.setMode(MCP_NORMAL);
    sendHeartbeatFrame(0x00);
}

unsigned long lastHeartbeatTime = 0;
unsigned long lastPdoTime = 0;
const uint16_t HEARTBEAT_PERIOD_MS = 1000;
const uint16_t PDO_PERIOD_MS = 2500; // deliberately off any round SYNC-multiple boundary - makes it
                                      // easy to visually tell the free-running and SYNC-triggered
                                      // TPDO1 sends apart in the trace, rather than have them land
                                      // suspiciously close together and look coincidentally coupled

void loop() {
    if (millis() - lastHeartbeatTime >= HEARTBEAT_PERIOD_MS) {
        sendHeartbeatFrame(currentState);
        lastHeartbeatTime = millis();
    }

    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        logCanFrame("RX", rxId, len, rxBuf);

        switch (classify(rxId)) {
            case MsgClass::Nmt:   handleNmtMessage(len, rxBuf); break;
            case MsgClass::Sync:  handleSyncMessage(len, rxBuf); break;
            case MsgClass::Emcy:  handleEmcyMessage(len, rxBuf); break;
            case MsgClass::Time:  handleTimeMessage(len, rxBuf); break;
            case MsgClass::Pdo:   handlePdoMessage(len, rxBuf); break;
            case MsgClass::Sdo:   handleSdoMessage(rxId, len, rxBuf); break;
            case MsgClass::Hb:    handleHbMessage(len, rxBuf); break;
            case MsgClass::Other: handleOtherMessage(len, rxBuf); break;
        }
    }

    if (currentState == OPERATIONAL) {
        if (millis() - lastPdoTime >= PDO_PERIOD_MS) {
            sendPdo1Frame();
            lastPdoTime = millis();
        }
    }

    digitalWrite(LED_PIN, od_led_state ? HIGH : LOW);
}

int main(void) {
    init();
    setup();
    for (;;) {
        loop();
    }
    return 0;
}
