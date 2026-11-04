/* align.c - BNO080 heading over I2C (SHTP), polled by a 200 Hz core-0 task
 * Game Rotation Vector (0x08, no magnetometer -> immune to motor fields) for yaw,
 * calibrated gyro (0x02) for yaw rate. Own-goal check uses the map pose. */
#include "robot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_dev_handle_t imu;
static volatile float yaw_raw, rate;   /* 32-bit floats: atomic on Xtensa */
static float yaw0;
static uint8_t seq;

static float q14(const uint8_t *p) { return (int16_t)(p[1] << 8 | p[0]) / 16384.0f; }

static void feature(uint8_t id, uint32_t us)   /* SHTP Set Feature command on channel 2 */
{
    uint8_t p[21] = { 21, 0, 2, seq++, 0xFD, id, 0, 0, 0, us & 0xFF, (us >> 8) & 0xFF, (us >> 16) & 0xFF, us >> 24 };
    i2c_master_transmit(imu, p, sizeof p, 20);
}

static void imu_task(void *p)
{
    (void)p; uint8_t b[IMU_RD_LEN + 12];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
        if (i2c_master_receive(imu, b, IMU_RD_LEN, 5) != ESP_OK) continue;
        int len = (b[1] << 8 | b[0]) & 0x7FFF;
        if (b[2] != 3 || (b[1] & 0x80)) continue;             /* only fresh input-report packets */
        if (len > IMU_RD_LEN) len = IMU_RD_LEN;
        for (int k = 4; k < len;) switch (b[k]) {
            case 0xFB: k += 5; break;                          /* base timestamp */
            case 0x08: { float x = q14(b + k + 4), y = q14(b + k + 6), z = q14(b + k + 8), w = q14(b + k + 10);
                         yaw_raw = IMU_YAW_SIGN * atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); k += 12; break; }
            case 0x02: rate = IMU_YAW_SIGN * (int16_t)(b[k + 9] << 8 | b[k + 8]) / 512.0f; k += 10; break;
            default:   k = len;
        }
    }
}

void align_init(i2c_master_bus_handle_t bus)
{
    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR, .scl_speed_hz = I2C_HZ }, &imu);
    vTaskDelay(pdMS_TO_TICKS(300));                            /* BNO080 boot + advertisement */
    feature(0x08, IMU_PERIOD_MS * 1000); feature(0x02, IMU_PERIOD_MS * 1000);
    xTaskCreatePinnedToCore(imu_task, "imu", 2048, NULL, 6, NULL, 0);
}

void  align_zero(void) { yaw0 = yaw_raw; }
float align_yaw(void)  { return wrap_pi(yaw_raw - yaw0); }
float align_rate(void) { return rate; }

/* Own home base is at the world origin: refuse headings within HOME_EXCL_RAD of it. */
bool align_safe(float heading) { return fabsf(wrap_pi(heading - map_bearing(0, 0))) > HOME_EXCL_RAD; }
