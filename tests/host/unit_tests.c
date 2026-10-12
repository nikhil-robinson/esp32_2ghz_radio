/* Host unit tests for the pure modules: gcc -I... unit_tests.c <sources> -lm */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bandwidth.h"
#include "db_code.h"
#include "fir_design.h"
#include "frame.h"
#include "iq_pack.h"
#include "lo_plan.h"
#include "scan_plan.h"

static int failures;
#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            failures++;                                                 \
        }                                                               \
    } while (0)

static void test_lo_plan(void)
{
    lo_plan_t p = lo_plan(2437);
    CHECK(p.pll_mhz == 2437 && p.offset_khz == 0 && !p.alternate);
    p = lo_plan(1842);  /* 6/5 divider: PLL at 2210.4 MHz */
    CHECK(p.alternate && p.pll_mhz == 2210 && p.offset_khz == 400);
    p = lo_plan(2209);
    CHECK(p.alternate && p.pll_mhz == 2650 && p.offset_khz == 800);
    CHECK(!lo_plan(2210).alternate && !lo_plan(1841).alternate);
    CHECK(lo_is_wifi_channel(2412) && lo_is_wifi_channel(2472) && lo_is_wifi_channel(2484));
    CHECK(!lo_is_wifi_channel(2413) && !lo_is_wifi_channel(2477) && !lo_is_wifi_channel(2402));
}

static void test_bandwidth(void)
{
    CHECK(bandwidth_filter_code(0) == 0);
    CHECK(bandwidth_filter_code(69) == 0 && bandwidth_filter_code(200) == 0);
    CHECK(bandwidth_filter_code(21) == 32 && bandwidth_filter_code(13) == 60);
    CHECK(bandwidth_filter_code(5) == 60);
    unsigned previous = 0;
    for (unsigned mhz = 69; mhz >= 13; mhz--) {  /* narrower bandwidth never lowers the code */
        unsigned code = bandwidth_filter_code(mhz);
        CHECK(code >= previous);
        previous = code;
    }
}

static uint32_t word(int i, int q) { return ((uint32_t)i & 0x3ffu) | (((uint32_t)q & 0x3ffu) << 10) | 0xab00000u; }

static void test_iq_pack(void)
{
    enum { N = 7 };
    uint32_t words[N], original[N];
    for (int j = 0; j < N; j++) {
        original[j] = words[j] = word(j * 73 - 256, 511 - j * 97);
    }
    size_t bytes = iq_pack10(words, N);
    CHECK(bytes == iq_pack10_size(N) && bytes == 18);
    const uint8_t *b = (const uint8_t *)words;
    for (int j = 0; j < N; j++) {  /* unpack exactly like the host decoder */
        const uint8_t *g = b + (j / 2) * 5;
        uint32_t v = j % 2 == 0 ? (g[0] | g[1] << 8 | (g[2] & 15u) << 16)
                                : (g[2] >> 4 | (uint32_t)g[3] << 4 | (uint32_t)g[4] << 12);
        CHECK(v == (original[j] & 0xfffffu));
    }
    memcpy(words, original, sizeof(words));
    CHECK(iq_pack8(words, N) == 2 * N);
    for (int j = 0; j < N; j++) {
        CHECK((int8_t)b[2 * j] == (int8_t)(original[j] >> 2));
        CHECK((int8_t)b[2 * j + 1] == (int8_t)(original[j] >> 12));
    }
}

static void test_db_code(void)
{
    db_code_init();
    for (float power = 1.0f; power < 1e9f; power *= 3.7f) {
        union { float f; uint32_t u; } v = {.f = power};
        double want = 2.0 * 10.0 * log10(power);  /* 0.5 dB codes */
        uint8_t got = db_code(v.u, 0);
        if (want < 255) {
            CHECK(fabs(got - want) <= 0.6);
        }
    }
    union { float f; uint32_t u; } sum = {.f = 8.0f * 1000.0f};
    union { float f; uint32_t u; } one = {.f = 1000.0f};
    CHECK(abs(db_code(sum.u, db_mean_offset_q4(8)) - db_code(one.u, 0)) <= 1);  /* mean of 8 equal powers */
}

static void test_fir_design(void)
{
    int16_t taps[FIR_STAGE2_MAX_TAPS];
    fir_design_stage1(taps);
    long sum = 0;
    for (unsigned i = 0; i < FIR_STAGE1_TAPS; i++) {
        sum += taps[i];
        CHECK(taps[i] == taps[FIR_STAGE1_TAPS - 1 - i] || i >= 16);  /* symmetric part */
    }
    CHECK(labs(sum - 32768) <= 16);  /* unity DC gain in Q15 */
    for (unsigned d2 = 4; d2 <= 128; d2 *= 2) {  /* decimation 32..1024 */
        uint32_t n = fir_stage2_taps(d2);
        CHECK(n % 8 == 0 && n <= FIR_STAGE2_MAX_TAPS);
        fir_design_lowpass(taps, n, d2);
        sum = 0;
        for (uint32_t i = 0; i < n; i++) {
            sum += taps[i];
        }
        CHECK(labs(sum - 32768) <= 64);
    }
}

static int feed(frame_parser_t *p, const uint8_t *data, size_t n, frame_event_t *last)
{
    int ready = 0;
    for (size_t i = 0; i < n; i++) {
        frame_event_t e = frame_parser_push(p, data[i]);
        if (e != FRAME_NONE) {
            *last = e;
            ready += e == FRAME_READY;
        }
    }
    return ready;
}

static size_t encode(uint8_t *out, uint8_t type, uint8_t seq, const void *payload, uint32_t n)
{
    uint32_t crc = frame_write_header(out, type, seq, n);
    memcpy(out + 8, payload, n);
    crc = frame_crc32(crc, payload, n);
    memcpy(out + 8 + n, &crc, 4);
    return 12 + n;
}

static void test_frame_codec(void)
{
    uint8_t buf[256];
    const uint8_t payload[4] = {0x5d, 0x09, 0, 0};
    size_t n = encode(buf, 0x10, 7, payload, sizeof(payload));
    CHECK(buf[0] == 0xE5 && buf[1] == 0xD5);
    CHECK(frame_crc32(0, "123456789", 9) == 0xCBF43926u);  /* IEEE check value */

    frame_parser_t p;
    frame_event_t last = FRAME_NONE;
    frame_parser_reset(&p);
    uint8_t noisy[300] = {0x00, 0xE5, 0x11, 0xE5};  /* boot noise incl. a false magic start */
    memcpy(noisy + 4, buf, n);
    CHECK(feed(&p, noisy, 4 + n, &last) == 1 && last == FRAME_READY);
    CHECK(p.type == 0x10 && p.seq == 7 && p.length == 4 && memcmp(p.payload, payload, 4) == 0);

    buf[9] ^= 1;  /* corrupt payload */
    CHECK(feed(&p, buf, n, &last) == 0 && last == FRAME_BAD_CRC);
    buf[9] ^= 1;
    CHECK(feed(&p, buf, n, &last) == 1);  /* recovers on the next good frame */

    uint8_t big[8];
    (void)frame_write_header(big, 0x01, 1, PROTOCOL_MAX_REQUEST_PAYLOAD + 1);
    CHECK(feed(&p, big, 8, &last) == 0 && last == FRAME_TOO_LONG && !frame_parser_busy(&p));

    size_t a = encode(buf, 0x01, 1, NULL, 0), b = encode(buf + a, 0x13, 2, NULL, 0);
    CHECK(feed(&p, buf, a + b, &last) == 2);  /* back-to-back frames in one read */
}

static void test_scan_plan(void)
{
    scan_plan_t p = scan_plan(1842, 2790, 40000000);
    CHECK(p.offset_mhz == 2 && p.keep_mhz == 16 && p.first_lo_mhz == 1840 && p.steps == 60);
    CHECK(scan_lo_mhz(&p, 59) == 2784 && scan_plan_valid(&p, 1842, 1840, 2790));
    p = scan_plan(1841, 2790, 16000000);
    CHECK(p.offset_mhz == 1 && p.keep_mhz == 6 && p.steps == 159 && scan_plan_valid(&p, 1841, 1840, 2790));
    p = scan_plan(1844, 2790, 80000000);
    CHECK(p.offset_mhz == 4 && p.keep_mhz == 32 && p.steps == 30 && scan_plan_valid(&p, 1844, 1840, 2790));
    p = scan_plan(1840, 2790, 40000000);  /* first LO 1838: below the verified range */
    CHECK(!scan_plan_valid(&p, 1840, 1840, 2790));
    p = scan_plan(2400, 2810, 40000000);  /* last LO 2814 > 2790 */
    CHECK(!scan_plan_valid(&p, 2400, 1840, 2790));
    CHECK(scan_plan(2400, 2400, 40000000).steps == 0);
}

int main(void)
{
    test_scan_plan();
    test_lo_plan();
    test_bandwidth();
    test_iq_pack();
    test_db_code();
    test_fir_design();
    test_frame_codec();
    printf(failures ? "%d FAILED\n" : "all C unit tests passed\n", failures);
    return failures != 0;
}
