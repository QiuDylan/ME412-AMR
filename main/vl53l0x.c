/* vl53l0x.c - VL53L0X time-of-flight sensor behind a TCA9548A I2C mux (bench test, TOF_TEST = 1).
 * Bus: TOF_I2C_PORT on TOF_SDA_PIN / TOF_SCL_PIN. Mux at MUX_ADDR: writing (1 << ch) to it connects
 * channel ch, so every sensor can keep the default address 0x29 (no XSHUT wiring needed).
 * Sensor init is a port of Pololu's VL53L0X Arduino library (ST API register sequence); the timing
 * budget step is left out (default ~30 ms per reading, fine for a bench test).
 * Test: scan the bus -> select TOF_MUX_CH -> check model ID 0xEE -> init -> print mm at 5 Hz. */
#include <stdio.h>
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define T_MS 50                              /* I2C transaction timeout */

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t mux, tof;
static esp_err_t err;                        /* first error since last reset of this flag */
static uint8_t stop_var;

static void wr(uint8_t r, uint8_t v)
{
    esp_err_t e = i2c_master_transmit(tof, (uint8_t[]){ r, v }, 2, T_MS);
    if (!err) err = e;
}
static void wr16(uint8_t r, uint16_t v)
{
    esp_err_t e = i2c_master_transmit(tof, (uint8_t[]){ r, v >> 8, v & 0xFF }, 3, T_MS);
    if (!err) err = e;
}
static void rdn(uint8_t r, uint8_t *b, int n)
{
    esp_err_t e = i2c_master_transmit_receive(tof, &r, 1, b, n, T_MS);
    if (!err) err = e;
}
static uint8_t rd(uint8_t r) { uint8_t v = 0; rdn(r, &v, 1); return v; }
static uint16_t rd16(uint8_t r) { uint8_t b[2] = { 0 }; rdn(r, b, 2); return b[0] << 8 | b[1]; }

/* wait until (reg & mask) != 0 (or == 0), or give up. NACKs while waiting are tolerated (sensor
 * busy, or resetting after a brown-out) and counted, so a refused read doesn't abort the init. */
static int nacks;
static bool wait_bit(uint8_t r, uint8_t mask, bool want_set, int ms)
{
    int64_t t0 = now_ms();
    for (;;) {
        err = ESP_OK;
        uint8_t v = rd(r);
        if (err) nacks++;
        else if (((v & mask) != 0) == want_set) return true;
        if (now_ms() - t0 > ms) {
            if (nacks) printf("TOF: sensor refused %d reads while waiting on reg 0x%02X\n", nacks, r);
            if (!err) err = ESP_ERR_TIMEOUT;
            return false;
        }
        vTaskDelay(1);
    }
}

static void scan_bus(i2c_master_bus_handle_t b, const char *what)
{
    printf("TOF: scan %s:", what);
    int n = 0;
    for (int a = 0x08; a < 0x78; a++)
        if (i2c_master_probe(b, a, T_MS) == ESP_OK) { printf(" 0x%02X", a); n++; }
    printf(n ? "\n" : " nothing\n");
}
static void scan(const char *what) { scan_bus(bus, what); }

/* Before the I2C driver takes the pins: weak internal pull-downs (~45k) on SDA/SCL. A powered mux
 * board's pull-ups (~10k) win -> pin reads 1. Reads 0 -> that wire isn't reaching a powered mux
 * (wrong GPIO, broken wire / joint, no power) - or the board has no pull-ups on its input side. */
static __attribute__((unused)) void pin_check(void)
{
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << TOF_SDA_PIN | 1ULL << TOF_SCL_PIN,
        .mode = GPIO_MODE_INPUT, .pull_down_en = 1 });
    vTaskDelay(pdMS_TO_TICKS(10));
    int sda = gpio_get_level(TOF_SDA_PIN), scl = gpio_get_level(TOF_SCL_PIN);
    printf("TOF: pin check (ESP pull-downs on): SDA GPIO%d = %d, SCL GPIO%d = %d  -> %s\n",
           TOF_SDA_PIN, sda, TOF_SCL_PIN, scl,
           sda && scl ? "both pulled up: wires reach a powered board" :
           !sda && !scl ? "NO pull-up on either: mux unpowered, wrong GPIOs, or board has no pull-ups" :
           "only ONE pulled up: the 0 wire is broken / on the wrong pin");
    gpio_reset_pin(TOF_SDA_PIN); gpio_reset_pin(TOF_SCL_PIN);
}

static bool mux_select(int ch)
{
    return i2c_master_transmit(mux, (uint8_t[]){ ch < 0 ? 0 : 1 << ch }, 1, T_MS) == ESP_OK;
}

static bool spad_info(uint8_t *count, bool *aperture)
{
    wr(0x80, 0x01); wr(0xFF, 0x01); wr(0x00, 0x00);
    wr(0xFF, 0x06); wr(0x83, rd(0x83) | 0x04);
    wr(0xFF, 0x07); wr(0x81, 0x01);
    wr(0x80, 0x01);
    wr(0x94, 0x6B); wr(0x83, 0x00);
    if (!wait_bit(0x83, 0xFF, true, 500)) return false;
    wr(0x83, 0x01);
    uint8_t t = rd(0x92);
    *count = t & 0x7F; *aperture = t >> 7;
    wr(0x81, 0x00); wr(0xFF, 0x06);
    wr(0x83, rd(0x83) & ~0x04);
    wr(0xFF, 0x01); wr(0x00, 0x01);
    wr(0xFF, 0x00); wr(0x80, 0x00);
    return !err;
}

static bool ref_cal(uint8_t vhv)
{
    wr(0x00, 0x01 | vhv);                                  /* SYSRANGE_START */
    if (!wait_bit(0x13, 0x07, true, 500)) return false;    /* RESULT_INTERRUPT_STATUS */
    wr(0x0B, 0x01);                                        /* SYSTEM_INTERRUPT_CLEAR */
    wr(0x00, 0x00);
    return !err;
}

/* tuning table from ST's API (VL53L0X_DefaultTuningSettings), as {reg, value} pairs */
static const uint8_t tuning[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},{0x24,0x01},{0x25,0xFF},
    {0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},{0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},
    {0x31,0x04},{0x32,0x03},{0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},
    {0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},{0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},
    {0x78,0x21},{0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},{0xFF,0x01},{0x31,0x04},
    {0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},
    {0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00},
};

/* returns NULL on success, else the step that failed */
static const char *tof_init(void)
{
    err = ESP_OK;
    uint8_t id = rd(0xC0);
    if (err) return "read model ID (no reply from 0x29)";
    if (id != 0xEE) printf("TOF: model ID 0x%02X (expected 0xEE - not a VL53L0X?) - carrying on\n", id);

    wr(0x89, rd(0x89) | 0x01);                              /* I/O at 2V8 */
    wr(0x88, 0x00);
    wr(0x80, 0x01); wr(0xFF, 0x01); wr(0x00, 0x00);
    stop_var = rd(0x91);
    wr(0x00, 0x01); wr(0xFF, 0x00); wr(0x80, 0x00);
    wr(0x60, rd(0x60) | 0x12);                              /* disable MSRC + pre-range signal checks */
    wr16(0x44, 32);                                         /* signal rate limit 0.25 MCPS (Q9.7) */
    wr(0x01, 0xFF);                                         /* SYSTEM_SEQUENCE_CONFIG */
    if (err) return "basic setup";

    uint8_t cnt; bool ap;
    if (!spad_info(&cnt, &ap)) return "SPAD info";
    uint8_t map[6];
    rdn(0xB0, map, 6);
    wr(0xFF, 0x01); wr(0x4F, 0x00); wr(0x4E, 0x2C); wr(0xFF, 0x00); wr(0xB6, 0xB4);
    int first = ap ? 12 : 0, on = 0;
    for (int i = 0; i < 48; i++) {
        if (i < first || on == cnt) map[i / 8] &= ~(1 << (i % 8));
        else if (map[i / 8] >> (i % 8) & 1) on++;
    }
    uint8_t w[7] = { 0xB0 };
    for (int i = 0; i < 6; i++) w[i + 1] = map[i];
    esp_err_t e = i2c_master_transmit(tof, w, 7, T_MS);
    if (!err) err = e;
    if (err) return "SPAD map";

    for (size_t i = 0; i < sizeof tuning / sizeof tuning[0]; i++) wr(tuning[i][0], tuning[i][1]);
    if (err) return "tuning table";

    wr(0x0A, 0x04);                                         /* interrupt on new sample ready */
    wr(0x84, rd(0x84) & ~0x10);
    wr(0x0B, 0x01);
    wr(0x01, 0xE8);
    nacks = 0;
    wr(0x01, 0x01); if (!ref_cal(0x40)) return "VHV calibration";
    if (nacks) printf("TOF: VHV calibration OK, but the sensor refused %d reads during it (busy or reset?)\n", nacks);
    nacks = 0;
    wr(0x01, 0x02); if (!ref_cal(0x00)) return "phase calibration";
    if (nacks) printf("TOF: phase calibration OK, sensor refused %d reads during it\n", nacks);
    wr(0x01, 0xE8);

    /* start continuous back-to-back ranging */
    wr(0x80, 0x01); wr(0xFF, 0x01); wr(0x00, 0x00);
    wr(0x91, stop_var);
    wr(0x00, 0x01); wr(0xFF, 0x00); wr(0x80, 0x00);
    wr(0x00, 0x02);
    return err ? "start ranging" : NULL;
}

/* latest distance in mm; status 11 = valid. Returns false on timeout / I2C error. */
static bool tof_read(int *mm, int *status)
{
    err = ESP_OK;
    if (!wait_bit(0x13, 0x07, true, 200)) return false;
    *status = rd(0x14) >> 3 & 0x0F;
    *mm = rd16(0x14 + 10);
    wr(0x0B, 0x01);
    return !err;
}

/* ---------- VL53L1X (same 0x29 address, same "VL53L0/1XV2" breakout, 16-bit register index) ---------- */
static int chip;                             /* 0 = VL53L0X, 1 = VL53L1X */

static void l1_wr(uint16_t r, const uint8_t *v, int n)
{
    uint8_t b[2 + 96] = { r >> 8, r & 0xFF };
    for (int i = 0; i < n; i++) b[2 + i] = v[i];
    esp_err_t e = i2c_master_transmit(tof, b, n + 2, T_MS);
    if (!err) err = e;
}
static void l1_wr8(uint16_t r, uint8_t v) { l1_wr(r, &v, 1); }
static void l1_rd(uint16_t r, uint8_t *v, int n)
{
    esp_err_t e = i2c_master_transmit_receive(tof, (uint8_t[]){ r >> 8, r & 0xFF }, 2, v, n, T_MS);
    if (!err) err = e;
}

/* ST ULD VL51L1X_DEFAULT_CONFIGURATION, registers 0x2D..0x87 (same table as avoid.c) */
static const uint8_t l1_cfg[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
_Static_assert(sizeof l1_cfg == 0x87 - 0x2D + 1, "VL53L1X config length");

static bool l1_ready(int ms)                 /* GPIO__TIO_HV_STATUS bit 0 = new sample (default polarity) */
{
    int64_t t0 = now_ms(); uint8_t b = 0;
    for (;;) {
        l1_rd(0x31, &b, 1);
        if (err) return false;
        if (b & 1) return true;
        if (now_ms() - t0 > ms) return false;
        vTaskDelay(1);
    }
}

static const char *l1_init(void)
{
    uint8_t b[2] = { 0 };
    int64_t t0 = now_ms();
    do { l1_rd(0xE5, b, 1); vTaskDelay(1); } while (!err && !(b[0] & 1) && now_ms() - t0 < 500);  /* boot done */
    if (err || !(b[0] & 1)) return "VL53L1X boot";
    l1_wr(0x2D, l1_cfg, sizeof l1_cfg);
    l1_wr8(0x87, 0x40);                                    /* one dummy ranging (ULD SensorInit) */
    if (!l1_ready(500)) return "VL53L1X first measurement";
    l1_wr8(0x86, 0x01); l1_wr8(0x87, 0x00); l1_wr8(0x08, 0x09); l1_wr8(0x0B, 0x00);
    l1_wr(0x5E, (uint8_t[]){ TOF_TB_A >> 8, TOF_TB_A & 0xFF }, 2);   /* 33 ms timing budget, long mode */
    l1_wr(0x61, (uint8_t[]){ TOF_TB_B >> 8, TOF_TB_B & 0xFF }, 2);
    l1_rd(0xDE, b, 2);                                     /* inter-measurement period */
    uint32_t im = (uint32_t)(((b[0] << 8 | b[1]) & 0x3FF) * TOF_IM_MS * 1.075f);
    l1_wr(0x6C, (uint8_t[]){ im >> 24, im >> 16, im >> 8, im }, 4);
    l1_wr8(0x86, 0x01);
    l1_wr8(0x87, 0x40);                                    /* start continuous ranging */
    return err ? "VL53L1X start ranging" : NULL;
}

static bool l1_read(int *mm, int *status)
{
    err = ESP_OK;
    if (!l1_ready(200)) return false;
    uint8_t b[17];
    l1_rd(0x89, b, sizeof b);                              /* status ... range at +13 */
    *status = b[0] & 0x1F;                                 /* raw 9 = valid */
    *mm = b[13] << 8 | b[14];
    l1_wr8(0x86, 0x01);                                    /* clear -> next sample */
    return !err;
}

/* which chip is at 0x29? VL53L1X: 16-bit index 0x010F reads 0xEA 0xCC. Else assume VL53L0X. */
static const char *sensor_start(void)
{
    err = ESP_OK;
    uint8_t id[2] = { 0 };
    l1_rd(0x010F, id, 2);
    if (!err && id[0] == 0xEA && id[1] == 0xCC) {
        chip = 1; printf("TOF: chip is a VL53L1X (ID 0xEACC)\n");
        return l1_init();
    }
    printf("TOF: not a VL53L1X (I2C %s, ID 0x%02X%02X) - trying VL53L0X\n", esp_err_to_name(err), id[0], id[1]);
    chip = 0;
    return tof_init();
}

void tof_test_main(void)
{
    yb_init();                                              /* make sure the wheels are stopped */
#if NET_LOG
    net_init();
#endif
#if TOF_USE_YB_BUS
    bus = yb_bus();                                         /* mux shares the Yahboom / MPU-6050 bus */
    if (!bus) { printf("TOF: motor bus not up - nothing to test\n"); for (;;) vTaskDelay(portMAX_DELAY); }
    printf("TOF TEST: mux on the motor bus (SDA %d / SCL %d), sensor on channel %d\n", YB_SDA_PIN, YB_SCL_PIN, TOF_MUX_CH);
#else
    printf("TOF TEST: mux on SDA %d / SCL %d, sensor on channel %d\n", TOF_SDA_PIN, TOF_SCL_PIN, TOF_MUX_CH);
    if (yb_bus()) scan_bus(yb_bus(), "motor bus GPIO10/11 (expect 0x26 0x68) - proves the scan works");
    pin_check();
    i2c_new_master_bus(&(i2c_master_bus_config_t){ .i2c_port = TOF_I2C_PORT, .sda_io_num = TOF_SDA_PIN,
        .scl_io_num = TOF_SCL_PIN, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1 }, &bus);
#endif
    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDR, .scl_speed_hz = TOF_I2C_HZ }, &tof);

    for (;;) {                                              /* retry every 2 s so wiring can be fixed live */
        scan("no channel selected (expect the mux, 0x70-0x77)");
        if (!mux) {                                         /* find the mux: 0x70 + A2 A1 A0 */
            for (int a = 0x70; a <= 0x77 && !mux; a++)
                if (i2c_master_probe(bus, a, T_MS) == ESP_OK) {
                    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                        .device_address = a, .scl_speed_hz = TOF_I2C_HZ }, &mux);
                    printf("TOF: mux found at 0x%02X%s\n", a, a == MUX_ADDR ? "" :
                           " - not 0x70: tie A0, A1, A2 to GND (floating address pins can change)");
                }
        }
        if (mux && !mux_select(TOF_MUX_CH)) {             /* was there, now gone: search again next time */
            printf("TOF: mux STOPPED answering - loose wire / floating RST or A0-A2?\n");
            i2c_master_bus_rm_device(mux); mux = NULL;
        } else if (!mux && i2c_master_probe(bus, VL53L0X_ADDR, T_MS) == ESP_OK) {
            printf("TOF: no mux, but the VL53L0X answers directly on this bus - ranging without the mux\n");
            const char *fail = sensor_start();
            if (!fail) break;
            printf("TOF: sensor init failed at: %s (I2C %s)\n", fail, esp_err_to_name(err));
        } else if (!mux) {
            printf("TOF: no mux answering at 0x70-0x77 - check SDA/SCL/3V3/GND, RST -> 3V3\n");
        } else {
            char s[48]; snprintf(s, sizeof s, "channel %d selected (expect + 0x29)", TOF_MUX_CH);
            scan(s);
            const char *fail = sensor_start();
            if (!fail) break;
            printf("TOF: sensor init failed at: %s (I2C %s) - check the sensor's wiring on SD%d/SC%d\n",
                   fail, esp_err_to_name(err), TOF_MUX_CH, TOF_MUX_CH);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    printf("TOF: ranging. Status %d = good reading%s\n", chip ? 9 : 11, chip ? "" : "; ~8190 mm = nothing in range");

    int bad = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        int mm, st;
        if (chip ? l1_read(&mm, &st) : tof_read(&mm, &st)) {
            printf("TOF %s%d: %4d mm  (status %d%s)\n", mux ? "ch" : "direct", mux ? TOF_MUX_CH : 0, mm, st, st == (chip ? 9 : 11) ? " ok" : "");
            bad = 0;
        } else if (++bad % 5 == 1) {
            printf("TOF: read failed (%s)%s\n", esp_err_to_name(err), mux ? " - re-selecting mux channel" : "");
            if (mux) mux_select(TOF_MUX_CH);
        }
    }
}
