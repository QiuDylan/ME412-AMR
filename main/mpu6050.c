/* mpu6050.c - MPU-6050 gyro-Z heading on the Yahboom's I2C bus (new i2c_master API).
 * A background task reads the Z-axis rate every tick (10 ms), removes the offset measured at boot
 * and integrates it into a heading in degrees (+ = CCW seen from above). There is no magnetometer,
 * so the heading drifts slowly (a few deg/min) - fine for short turns, re-zero between uses.
 * Registers: 0x6B PWR_MGMT_1, 0x1A CONFIG (DLPF), 0x1B GYRO_CONFIG, 0x47 GYRO_ZOUT_H, 0x75 WHO_AM_I. */
#include <stdio.h>
#include "robot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LSB_PER_DPS 65.5f                    /* +-500 deg/s full scale */

static i2c_master_dev_handle_t mpu;
static float bias;                           /* raw gyro-Z offset at rest */
static volatile float yaw;                   /* degrees; 32-bit store is atomic on ESP32 */

static esp_err_t wr(uint8_t reg, uint8_t v)
{
    return i2c_master_transmit(mpu, (uint8_t[]){ reg, v }, 2, YB_TIMEOUT_MS);
}

static esp_err_t gz_raw(int16_t *g)
{
    uint8_t reg = 0x47, b[2];
    esp_err_t e = i2c_master_transmit_receive(mpu, &reg, 1, b, 2, YB_TIMEOUT_MS);
    if (e == ESP_OK) *g = (int16_t)(b[0] << 8 | b[1]);
    return e;
}

static void mpu_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    int64_t t0 = esp_timer_get_time();
    int errs = 0;
    for (;;) {
        vTaskDelayUntil(&wake, 1);
        int64_t t = esp_timer_get_time();
        float dt = (t - t0) * 1e-6f; t0 = t;
        int16_t g;
        if (gz_raw(&g) != ESP_OK) {          /* missed sample: heading just loses this slice */
            if (++errs % 100 == 1) printf("MPU: read error (%d so far)\n", errs);
            continue;
        }
        float dps = (g - bias) / LSB_PER_DPS;
        if (fabsf(dps) < MPU_DEADBAND_DPS) dps = 0;
        yaw += MPU_YAW_SIGN * dps * dt;
    }
}

bool mpu_init(i2c_master_bus_handle_t bus)
{
    uint8_t reg = 0x75, who = 0;
    if (!bus || i2c_master_probe(bus, MPU_ADDR, 50) != ESP_OK) {
        printf("MPU-6050 not found at 0x%02X (SDA %d, SCL %d) - check wiring / AD0\n", MPU_ADDR, YB_SDA_PIN, YB_SCL_PIN);
        return false;
    }
    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR, .scl_speed_hz = YB_HZ }, &mpu);
    i2c_master_transmit_receive(mpu, &reg, 1, &who, 1, YB_TIMEOUT_MS);
    if (who != 0x68) printf("MPU: WHO_AM_I = 0x%02X (expected 0x68; clone chips differ - carrying on)\n", who);

    wr(0x6B, 0x80); vTaskDelay(pdMS_TO_TICKS(100));       /* reset */
    wr(0x6B, 0x01); vTaskDelay(pdMS_TO_TICKS(50));        /* wake, clock = gyro X PLL */
    wr(0x1A, MPU_DLPF);
    wr(0x1B, 0x08);                                       /* +-500 deg/s */
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("MPU: keep the robot still - measuring gyro offset...\n");
    int32_t sum = 0, n = 0, lo = 32767, hi = -32768;
    for (int i = 0; i < MPU_BIAS_SAMPLES; i++) {
        int16_t g;
        if (gz_raw(&g) == ESP_OK) { sum += g; n++; if (g < lo) lo = g; if (g > hi) hi = g; }
        vTaskDelay(1);
    }
    if (!n) { printf("MPU: no gyro readings\n"); return false; }
    bias = (float)sum / n;
    printf("MPU: offset %.1f deg/s  (spread %.2f deg/s%s)\n", (double)(bias / LSB_PER_DPS),
           (double)((hi - lo) / LSB_PER_DPS), (hi - lo) / LSB_PER_DPS > 2.0f ? " - was it moving? reboot to redo" : "");
    yaw = 0;
    xTaskCreate(mpu_task, "mpu", 3072, NULL, 5, NULL);
    return true;
}

float mpu_yaw_deg(void) { return yaw; }
