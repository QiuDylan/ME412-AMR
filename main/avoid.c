/* avoid.c - VL53L1X time-of-flight ring (register-level, ST ULD sequence) + reflexive avoidance
 * A core-0 task polls all sensors; the control loop only reads cached ranges (no I2C wait). */
#include <string.h>
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int xs[N_TOF] = TOF_XSHUT_PINS, avoid_mm[N_TOF] = AVOID_MM;
static const float ta[N_TOF] = TOF_ANGLES;
static float ux[N_TOF], uy[N_TOF];
static i2c_master_dev_handle_t tof[N_TOF];
static volatile int range[N_TOF];
static volatile uint8_t fresh[N_TOF];

/* VL53L1X default configuration, registers 0x2D..0x87 (from ST ULD VL51L1X_DEFAULT_CONFIGURATION) */
static const uint8_t cfg[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
_Static_assert(sizeof cfg == 0x87 - 0x2D + 1, "VL53L1X config length");

static void wr(i2c_master_dev_handle_t d, uint16_t reg, const uint8_t *v, size_t n)
{
    uint8_t b[2 + sizeof cfg] = { reg >> 8, reg & 0xFF }; memcpy(b + 2, v, n);
    i2c_master_transmit(d, b, n + 2, 10);
}
static void wr8(i2c_master_dev_handle_t d, uint16_t reg, uint8_t v) { wr(d, reg, &v, 1); }
static void rd(i2c_master_dev_handle_t d, uint16_t reg, uint8_t *v, size_t n)
{
    uint8_t a[2] = { reg >> 8, reg & 0xFF }; i2c_master_transmit_receive(d, a, 2, v, n, 10);
}

static void tof_setup(i2c_master_dev_handle_t d)
{
    uint8_t b[2] = { 0 };
    wr(d, 0x2D, cfg, sizeof cfg);
    wr8(d, 0x87, 0x40);                                                        /* one dummy ranging (ULD SensorInit) */
    for (int k = 0; k < 50 && (rd(d, 0x31, b, 1), !(b[0] & 1)); k++) vTaskDelay(pdMS_TO_TICKS(2));
    wr8(d, 0x86, 0x01); wr8(d, 0x87, 0x00); wr8(d, 0x08, 0x09); wr8(d, 0x0B, 0x00);
    wr(d, 0x5E, (uint8_t[]){ TOF_TB_A >> 8, TOF_TB_A & 0xFF }, 2);             /* timing budget */
    wr(d, 0x61, (uint8_t[]){ TOF_TB_B >> 8, TOF_TB_B & 0xFF }, 2);
    rd(d, 0xDE, b, 2);                                                         /* inter-measurement period */
    uint32_t im = (uint32_t)(((b[0] << 8 | b[1]) & 0x3FF) * TOF_IM_MS * 1.075f);
    wr(d, 0x6C, (uint8_t[]){ im >> 24, im >> 16, im >> 8, im }, 4);
    wr8(d, 0x87, 0x40);                                                        /* start continuous ranging */
}

static void tof_task(void *p)
{
    (void)p; uint8_t b[17], last[N_TOF] = { 0 };
    for (;;) {
        for (int i = 0; i < N_TOF; i++) {
            rd(tof[i], 0x89, b, sizeof b);                 /* status, stream count, ..., range @13 */
            if (b[2] == last[i]) continue;                 /* no new measurement */
            last[i] = b[2];
            range[i] = ((b[0] & 0x1F) == 9) ? (b[13] << 8 | b[14]) : TOF_MAX_MM;
            fresh[i] = 1;
            wr8(tof[i], 0x86, 0x01);                       /* clear interrupt -> next measurement */
        }
        vTaskDelay(pdMS_TO_TICKS(TOF_POLL_MS));
    }
}

/* All XSHUTs low, then wake one at a time and move each from 0x29 to TOF_ADDR0 + i. */
void avoid_init(i2c_master_bus_handle_t bus)
{
    for (int i = 0; i < N_TOF; i++) {
        ux[i] = cosf(ta[i]); uy[i] = sinf(ta[i]); range[i] = TOF_MAX_MM;
        if (xs[i] >= 0) { gpio_set_direction(xs[i], GPIO_MODE_OUTPUT); gpio_set_level(xs[i], 0); }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    for (int i = 0; i < N_TOF; i++) {
        i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x29, .scl_speed_hz = I2C_HZ };
        if (xs[i] >= 0) gpio_set_level(xs[i], 1);
        vTaskDelay(pdMS_TO_TICKS(5));                      /* boot time */
        i2c_master_bus_add_device(bus, &dc, &tof[i]);
        wr8(tof[i], 0x0001, TOF_ADDR0 + i);
        i2c_master_bus_rm_device(tof[i]);
        dc.device_address = TOF_ADDR0 + i;
        i2c_master_bus_add_device(bus, &dc, &tof[i]);
        tof_setup(tof[i]);
    }
    xTaskCreatePinnedToCore(tof_task, "tof", 3072, NULL, 5, NULL, 0);
}

int tof_fresh(int i) { if (!fresh[i]) return -1; fresh[i] = 0; return range[i]; }

/* 1) scale forward speed down between SLOW_MM and STOP_MM (negative = back off)
 * 2) repulsive vector from every sensor inside its AVOID_MM envelope
 * Only translation is overridden, so the hull can keep rotating toward its target. */
bool avoid_cmd(cmd_t *c)
{
    float fx = 0, fy = 0; int f = range[TOF_F];
    if (c->vx > 0 && f < SLOW_MM) c->vx *= (float)(f - STOP_MM) / (SLOW_MM - STOP_MM);
    for (int i = 0; i < N_TOF; i++) {
        int d = range[i];
        if (d < avoid_mm[i]) { float k = (float)(avoid_mm[i] - d) / avoid_mm[i]; fx -= ux[i] * k; fy -= uy[i] * k; }
    }
    if (fx == 0 && fy == 0) return false;
    if (fx < 0) fy += (range[TOF_L] > range[TOF_R]) ? SIDESTEP : -SIDESTEP;   /* sidestep to open side */
    c->vx = fx * V_AVOID; c->vy = fy * V_AVOID;
    return true;
}
