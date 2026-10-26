/* align.c - IMU heading (gyro-Z integration in a 200 Hz task) + own-base safety check */
#include "robot.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static i2c_master_dev_handle_t imu;
static volatile float yaw, rate;   /* 32-bit floats: atomic reads on Xtensa */
static float bias;

static float gyro_z_dps(void)
{
    uint8_t reg = IMU_REG_GYRO_Z, b[2] = { 0 };
    i2c_master_transmit_receive(imu, &reg, 1, b, 2, 5);
    return (int16_t)(b[0] << 8 | b[1]) / IMU_LSB_PER_DPS;
}

static void imu_task(void *p)
{
    (void)p; TickType_t wake = xTaskGetTickCount(); int64_t t0 = esp_timer_get_time();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
        int64_t t = esp_timer_get_time();
        rate = IMU_YAW_SIGN * (gyro_z_dps() - bias) * DEG(1);
        yaw = wrap_pi(yaw + rate * (float)(t - t0) * 1e-6f); t0 = t;
    }
}

void align_init(void)
{
    i2c_master_bus_handle_t bus;
    i2c_new_master_bus(&(i2c_master_bus_config_t){ .i2c_port = I2C_NUM_0, .sda_io_num = I2C_SDA_PIN, .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = 1 }, &bus);
    i2c_master_bus_add_device(bus, &(i2c_device_config_t){ .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR, .scl_speed_hz = I2C_HZ }, &imu);
    static const uint8_t seq[][2] = IMU_INIT_SEQ;
    for (size_t i = 0; i < sizeof seq / sizeof seq[0]; i++) i2c_master_transmit(imu, seq[i], 2, 5);
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = 0; i < IMU_CAL_SAMPLES; i++) { bias += gyro_z_dps(); esp_rom_delay_us(2000); }
    bias /= IMU_CAL_SAMPLES;
    xTaskCreatePinnedToCore(imu_task, "imu", 2048, NULL, 6, NULL, 0);
}

void  align_zero(void) { yaw = 0; }
float align_yaw(void)  { return yaw; }
float align_rate(void) { return rate; }

/* Home base lies at yaw = pi (robot starts facing away from it). */
bool align_safe(float heading) { return fabsf(wrap_pi(heading - (float)M_PI)) > HOME_EXCL_RAD; }

