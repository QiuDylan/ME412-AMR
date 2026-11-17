/* yahboom.c - Yahboom 4-ch encoder motor driver (STM32 co-processor, I2C 0x26) on its own I2C bus.
 * Uses the new i2c_master API (ESP-IDF aborts at boot if the legacy driver/i2c.h is mixed in).
 * Wheel order everywhere in this file's API is FL, FR, RL, RR (same as drive.c); YB_WHEEL_MAP
 * routes each wheel to the driver's M1..M4 and YB_DIR makes + = wheel drives the robot forward.
 * Registers: 0x01 type, 0x02 deadzone, 0x03 encoder lines, 0x04 ratio, 0x05 wheel dia (float),
 *            0x06 speed 4x int16 BE, 0x20..0x27 total encoder counts (hi/lo word per motor). */
#include <stdio.h>
#include <string.h>
#include "robot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_dev_handle_t yb;
static volatile int64_t t_cmd;              /* last yb_wheels() call */
static volatile bool moving;                /* last command sent was non-zero */

static const int wmap[4] = YB_WHEEL_MAP, wdir[4] = YB_DIR;

static esp_err_t wr(uint8_t reg, const void *d, size_t n)
{
    uint8_t b[9] = { reg }; memcpy(b + 1, d, n);
    return i2c_master_transmit(yb, b, n + 1, YB_TIMEOUT_MS);
}
static void wr16(uint8_t reg, uint16_t v)   /* config writes, big-endian; driver needs time between them */
{
    wr(reg, (uint8_t[]){ v >> 8, v & 0xFF }, 2); vTaskDelay(pdMS_TO_TICKS(100));
}

/* The Yahboom's I2C bus (created on first call). The MPU-6050 shares it at 0x68. */
i2c_master_bus_handle_t yb_bus(void)
{
    static i2c_master_bus_handle_t bus;
    if (!bus && i2c_new_master_bus(&(i2c_master_bus_config_t){ .i2c_port = YB_I2C_PORT, .sda_io_num = YB_SDA_PIN,
            .scl_io_num = YB_SCL_PIN, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = 1 }, &bus) != ESP_OK) {
        printf("I2C bus on SDA %d / SCL %d failed to start\n", YB_SDA_PIN, YB_SCL_PIN);
        bus = NULL;
    }
    return bus;
}

/* The Yahboom has no command timeout. If our code stops refreshing a non-zero command (hang, stuck
 * loop), zero the wheels. Anything that drives must resend its command at least every YB_WATCHDOG_MS. */
static void watchdog_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (moving && now_ms() - t_cmd > YB_WATCHDOG_MS) {
            yb_wheels((const int16_t[4]){ 0 });
            printf("Yahboom: WATCHDOG STOP - no wheel command for %d ms (control loop stuck?)\n", YB_WATCHDOG_MS);
        }
    }
}

bool yb_init(void)
{
    i2c_master_bus_handle_t bus = yb_bus();
    bool found = false;
    for (int k = 0; bus && !found && k < 30; k++) {           /* its STM32 may still be booting: retry ~3 s */
        found = i2c_master_probe(bus, YB_ADDR, 50) == ESP_OK;
        if (!found) vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!found) {
        printf("Yahboom driver not found at 0x%02X (SDA %d, SCL %d) - check wiring/12V\n", YB_ADDR, YB_SDA_PIN, YB_SCL_PIN);
        return false;
    }
    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = YB_ADDR, .scl_speed_hz = YB_HZ }, &yb);
    yb_brake();                              /* first thing: the driver keeps its last command (and its speed-PID */
    yb_wheels((const int16_t[4]){ 0 });     /* state) across our resets; PWM 0 clears it - it fixed a runaway M3 */
    wr(0x01, (uint8_t[]){ YB_MOTOR_TYPE, 0 }, 2); vTaskDelay(pdMS_TO_TICKS(100));
    wr16(0x04, YB_GEAR_RATIO);
    wr16(0x03, YB_ENC_LINES);
    float d = YB_WHEEL_MM; wr(0x05, &d, 4); vTaskDelay(pdMS_TO_TICKS(100));   /* raw LE float, as Yahboom's code */
    wr16(0x02, YB_DEADZONE);
    yb_wheels((const int16_t[4]){ 0 });
    if (MOTORS_OFF) printf("Yahboom: MOTORS_OFF = 1 - wheels disabled\n");
    xTaskCreate(watchdog_task, "yb_wd", 3072, NULL, 6, NULL);
    return true;
}

/* Closed-loop wheel speeds, Yahboom units -1000..1000, order FL, FR, RL, RR. */
void yb_wheels(const int16_t w[4])
{
    if (!yb) return;                                          /* driver not found at boot */
    uint8_t b[8]; bool nz = false;
    for (int i = 0; i < 4; i++) {
        int16_t v = MOTORS_OFF || (NET_LOG && net_stopped()) ? 0 : (int16_t)(wdir[i] * w[i]);
        b[2 * wmap[i]] = (uint8_t)(v >> 8); b[2 * wmap[i] + 1] = (uint8_t)v;
        nz |= v != 0;
    }
    t_cmd = now_ms(); moving = nz;
    static int errs;
    for (int k = 0; k < 3; k++) if (wr(0x06, b, 8) == ESP_OK) return;   /* retry: a lost stop = robot keeps going */
    if (errs++ % 50 == 0) printf("Yahboom: speed write failed (%d so far)\n", errs);
}

/* Hard stop: open-loop PWM 0 on all four motors (register 0x07), bypassing the driver's speed PID.
 * The next yb_wheels() call puts it back in closed-loop speed mode. */
void yb_brake(void)
{
    if (!yb) return;
    yb_wheels((const int16_t[4]){ 0 });     /* speed target 0 first, so the driver's PID isn't left winding up */
    for (int k = 0; k < 3; k++) if (wr(0x07, (uint8_t[8]){ 0 }, 8) == ESP_OK) break;
    moving = false;
}

/* Spin in place, + = CCW (same wheel signs as drive.c with w > 0). */
void yb_spin(int s) { yb_wheels((const int16_t[4]){ (int16_t)-s, (int16_t)s, (int16_t)-s, (int16_t)s }); }

/* Total encoder counts since power-up, order FL, FR, RL, RR, + = forward. */
esp_err_t yb_counts(int32_t c[4])
{
    if (!yb) return ESP_ERR_INVALID_STATE;
    int32_t m[4];
    for (int j = 0; j < 4; j++) {
        uint8_t hr = (uint8_t)(0x20 + 2 * j), lr = hr + 1, h[2], l[2];
        esp_err_t e = i2c_master_transmit_receive(yb, &hr, 1, h, 2, YB_TIMEOUT_MS);
        if (e == ESP_OK) e = i2c_master_transmit_receive(yb, &lr, 1, l, 2, YB_TIMEOUT_MS);
        if (e != ESP_OK) return e;
        m[j] = (int32_t)((uint32_t)h[0] << 24 | (uint32_t)h[1] << 16 | (uint32_t)l[0] << 8 | l[1]);
    }
    for (int i = 0; i < 4; i++) c[i] = YB_ENC_SIGN * wdir[i] * m[wmap[i]];
    return ESP_OK;
}
