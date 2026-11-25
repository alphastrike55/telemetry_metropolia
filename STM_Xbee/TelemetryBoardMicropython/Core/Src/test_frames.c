/*
 * test_frames.c
 *
 * Sample CAN frames (example IDs and layouts, not the real car DBC) and a
 * random batch sender that produces exactly the JSON the server expects:
 *
 *   {
 *     "seq":    uint32, +1 per published batch, separate counter per bus
 *     "ts":     uint64, batch timestamp, Unix epoch milliseconds (UTC)
 *     "frames": [ { "id": CAN ID as decimal, "data": uppercase hex, no 0x, no spaces } ]
 *   }
 *
 * The topic is chosen by bus: HPF26/Telemetry/Can_1 or HPF26/Telemetry/Can_2.
 * All multi-byte signals below are little-endian (Intel).
 */

#include "test_frames.h"

#define TEST_TOPIC_PREFIX          "HPF26/Telemetry/Can_"
#define TEST_MAX_FRAMES_PER_BATCH  8u
#define TEST_LINE_BUF_SIZE         512u
#define TEST_UART_TIMEOUT_MS       100u

/* There is no real-time clock yet: ts = this base + HAL_GetTick().
 * 1795219200000 = 2026-11-21 00:00:00 UTC. Replace with network time later. */
#define TEST_EPOCH_BASE_MS         1795219200000ULL

typedef struct {
    uint8_t  bus;      /* 1 = Can_1, 2 = Can_2 */
    uint32_t id;       /* CAN ID, 11 or 29 bit */
    uint8_t  len;      /* data bytes, 0..8 */
    uint8_t  data[8];
} test_frame_t;

static const test_frame_t test_frames[] = {
    /* ---------------- Can_1: powertrain (inverters, VCU) ---------------- */

    /* 0x0A0 (160) INV_FL_ACTUAL, every 5 ms
     * [0..1] speed int16 rpm         = 12000 rpm
     * [2..3] torque int16 x0.01 Nm   = 18.50 Nm
     * [4..5] DC bus uint16 x0.1 V    = 548.6 V
     * [6]    status bits: b0 ready, b1 on, b2 derating, b3 error = ready + on
     * [7]    alive counter           = 0x5A */
    { 1, 0x0A0, 8, {0xE0, 0x2E, 0x3A, 0x07, 0x6E, 0x15, 0x03, 0x5A} },

    /* 0x0A1 (161) INV_FL_TEMPS, every 100 ms
     * [0..1] motor temp int16 x0.1 C = 67.4 C
     * [2..3] IGBT temp int16 x0.1 C  = 52.1 C
     * [4..5] cold plate int16 x0.1 C = 38.0 C
     * [6..7] error code uint16       = 0 (no error) */
    { 1, 0x0A1, 8, {0xA2, 0x02, 0x09, 0x02, 0x7C, 0x01, 0x00, 0x00} },

    /* 0x0A4 (164) INV_FR_ACTUAL, every 5 ms, same layout as 0x0A0
     * 11985 rpm, 17.95 Nm, 548.4 V, ready + on, counter 0x5B */
    { 1, 0x0A4, 8, {0xD1, 0x2E, 0x03, 0x07, 0x6C, 0x15, 0x03, 0x5B} },

    /* 0x0A8 (168) INV_FL_FAULT, event-driven, DLC 5 (shorter than 8 on purpose)
     * [0..1] error code uint16       = 2310 (overtemperature warning)
     * [2]    severity: 1 warning, 2 error = warning
     * [3..4] motor temp int16 x0.1 C = 120.5 C at the moment of the event */
    { 1, 0x0A8, 5, {0x06, 0x09, 0x01, 0xB5, 0x04} },

    /* 0x100 (256) VCU_PEDALS, every 10 ms
     * [0..1] APPS1 uint16 x0.1 %     = 75.2 %
     * [2..3] APPS2 uint16 x0.1 %     = 75.0 %
     * [4..5] brake front x0.01 bar   = 0.35 bar
     * [6..7] brake rear x0.01 bar    = 0.20 bar */
    { 1, 0x100, 8, {0xF0, 0x02, 0xEE, 0x02, 0x23, 0x00, 0x14, 0x00} },

    /* 0x101 (257) VCU_STATE, every 10 ms
     * [0]    state: 0 idle, 1 precharge, 2 HV on, 3 ready to drive = 3
     * [1..2] torque request int16 x0.01 Nm = 20.00 Nm
     * [3]    mode: 0 accel, 1 skidpad, 2 autocross, 3 endurance = endurance
     * [4]    flags: b0 TC, b1 TV, b2 regen = TC + regen
     * [5]    power limit kW          = 80
     * [6]    reserved
     * [7]    alive counter           = 0x12 */
    { 1, 0x101, 8, {0x03, 0xD0, 0x07, 0x03, 0x05, 0x50, 0x00, 0x12} },

    /* ---------------- Can_2: vehicle (BMS, sensors) ---------------- */

    /* 0x300 (768) BMS_CELLS, multiplexed, one group every 10 ms
     * [0]    mux: group 4 = cells 12..14
     * [1..2] cell 12 uint16 mV       = 4105 mV
     * [3..4] cell 13 uint16 mV       = 4098 mV
     * [5..6] cell 14 uint16 mV       = 4111 mV
     * [7]    group max temp C        = 34 C */
    { 2, 0x300, 8, {0x04, 0x09, 0x10, 0x02, 0x10, 0x0F, 0x10, 0x22} },

    /* 0x310 (784) BMS_PACK, every 100 ms
     * [0..1] pack voltage uint16 x0.1 V = 548.9 V
     * [2..3] current int16 x0.1 A (+ = discharge) = 123.4 A
     * [4]    SoC %                   = 76 %
     * [5]    max cell temp C         = 41 C
     * [6]    flags: b0 AIR+, b1 AIR-, b2 precharge done, b3 fault = AIR+ AIR- precharge
     * [7]    alive counter           = 0x33 */
    { 2, 0x310, 8, {0x71, 0x15, 0xD2, 0x04, 0x4C, 0x29, 0x07, 0x33} },

    /* 0x0B0 (176) WHEELS, every 10 ms, signals packed across byte borders
     * bits 0..11  FL x0.1 km/h = 123.4    bits 12..23 FR = 124.0
     * bits 24..35 RL           = 122.5    bits 36..47 RR = 123.1
     * bits 48..63 steering int16 x0.1 deg = -12.5 deg */
    { 2, 0x0B0, 8, {0xD2, 0x84, 0x4D, 0xC9, 0xF4, 0x4C, 0x83, 0xFF} },

    /* 0x0C0 (192) IMU, every 10 ms
     * [0..1] ax int16 x0.001 g       = +0.850 g
     * [2..3] ay int16 x0.001 g       = -1.230 g
     * [4..5] az int16 x0.001 g       = +1.010 g
     * [6..7] yaw rate int16 x0.1 deg/s = 35.6 deg/s */
    { 2, 0x0C0, 8, {0x52, 0x03, 0x32, 0xFB, 0xF2, 0x03, 0x64, 0x01} },
};

#define TEST_FRAME_COUNT  (sizeof(test_frames) / sizeof(test_frames[0]))

static uint32_t rng_state = 1u;
static uint32_t seq_per_bus[3];     /* [1] = Can_1, [2] = Can_2 */
static char     line_buf[TEST_LINE_BUF_SIZE];

/* xorshift32: small, fast, no stdlib needed */
static uint32_t rng_next(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

/* ---- minimal text builder (no printf: newlib-nano cannot print uint64) ---- */

typedef struct {
    char     *buf;
    uint32_t  len;
    uint32_t  cap;
    uint8_t   overflow;
} line_t;

static void put_char(line_t *l, char c)
{
    if (l->len + 1u < l->cap) {
        l->buf[l->len++] = c;
    } else {
        l->overflow = 1u;
    }
}

static void put_str(line_t *l, const char *s)
{
    while (*s != '\0') {
        put_char(l, *s++);
    }
}

static void put_u64(line_t *l, uint64_t v)
{
    char tmp[20];
    uint32_t n = 0u;
    do {
        tmp[n++] = (char)('0' + (uint32_t)(v % 10u));
        v /= 10u;
    } while (v != 0u);
    while (n != 0u) {
        put_char(l, tmp[--n]);
    }
}

static void put_hex(line_t *l, const uint8_t *data, uint8_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    for (uint8_t i = 0u; i < len; i++) {
        put_char(l, hex[data[i] >> 4]);
        put_char(l, hex[data[i] & 0x0Fu]);
    }
}

/* ---- public API ---- */

void TestFrames_Init(uint32_t seed)
{
    rng_state = (seed != 0u) ? seed : 0x12345678u;
    seq_per_bus[1] = 0u;
    seq_per_bus[2] = 0u;
}

HAL_StatusTypeDef TestFrames_SendRandomBatch(UART_HandleTypeDef *huart)
{
    uint8_t idx[TEST_FRAME_COUNT];
    uint8_t n = 0u;

    /* 1. random bus, collect the frames that belong to it */
    uint8_t bus = (uint8_t)(1u + rng_next() % 2u);
    for (uint8_t i = 0u; i < TEST_FRAME_COUNT; i++) {
        if (test_frames[i].bus == bus) {
            idx[n++] = i;
        }
    }
    if (n == 0u) {
        return HAL_ERROR;
    }

    /* 2. random number of frames, picked in random order (repeats allowed, like a real bus) */
    uint8_t count = (uint8_t)(1u + rng_next() % TEST_MAX_FRAMES_PER_BATCH);

    line_t l = { line_buf, 0u, TEST_LINE_BUF_SIZE, 0u };

    /* topic, space, JSON, newline */
    put_str(&l, TEST_TOPIC_PREFIX);
    put_u64(&l, bus);
    put_char(&l, ' ');

    put_str(&l, "{\"seq\":");
    put_u64(&l, seq_per_bus[bus]);
    put_str(&l, ",\"ts\":");
    put_u64(&l, TEST_EPOCH_BASE_MS + (uint64_t)HAL_GetTick());
    put_str(&l, ",\"frames\":[");

    for (uint8_t k = 0u; k < count; k++) {
        const test_frame_t *f = &test_frames[idx[rng_next() % n]];
        if (k != 0u) {
            put_char(&l, ',');
        }
        put_str(&l, "{\"id\":");
        put_u64(&l, f->id);
        put_str(&l, ",\"data\":\"");
        put_hex(&l, f->data, f->len);
        put_str(&l, "\"}");
    }
    put_str(&l, "]}\n");

    if (l.overflow != 0u) {
        return HAL_ERROR;
    }

    /* 3. send; count the batch only if it actually left the UART */
    HAL_StatusTypeDef st = HAL_UART_Transmit(huart, (uint8_t *)line_buf,
                                             (uint16_t)l.len, TEST_UART_TIMEOUT_MS);
    if (st == HAL_OK) {
        seq_per_bus[bus]++;
    }
    return st;
}
