/* config.h - ALL pinouts and tuning constants (Waveshare ESP32-S3)
 * Avoid: GPIO0/3/45/46 (strapping), 19/20 (USB), 26-37 (flash/PSRAM), 43/44 (UART0).
 * Verify every pin against your specific Waveshare board's pinout. */
#pragma once
#include "esp_adc/adc_oneshot.h"

/* ---------- Timing ---------- */
#define LOOP_MS            20        /* main control loop period (50 Hz) */
#define LEAVE_DOCK_MS      1500      /* blind drive-out time after start */
#define RUN_TIMEOUT_MS     60000     /* give up hunting and go home after this */

/* ---------- Mecanum drive: PWM + DIR per wheel, order FL, FR, RL, RR ---------- */
#define MOTOR_PWM_PINS     {9, 11, 13, 15}
#define MOTOR_DIR_PINS     {10, 12, 14, 16}
#define MOTOR_INVERT       {0, 1, 0, 1}  /* flip a wheel's direction without rewiring */
#define MOTOR_PWM_HZ       20000
#define MOTOR_PWM_BITS     LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX      1023
#define YAW_RATE_MAX       3.0f      /* rad/s at w = 1.0 (calibrate; or replace with IMU) */

/* Normalized speeds (0..1) and gains */
#define V_CRUISE           0.6f
#define V_APPROACH         0.4f
#define V_AVOID            0.7f
#define W_SEARCH           0.35f     /* spin rate while searching for IR */
#define KP_TRACK           0.012f    /* w per degree of IR bearing error */
#define KP_YAW             0.8f      /* w per rad of heading error (return home) */

/* ---------- Ultrasonic (HC-SR04): shared TRIG, one ECHO each (5V->3.3V divider!) ---------- */
enum { SON_F, SON_L, SON_R, SON_B, N_SONAR };
#define SONAR_TRIG_PIN     17
#define SONAR_ECHO_PINS    {18, 38, 39, 40}
#define SONAR_UX           {1, 0, 0, -1}     /* sensor facing unit vectors (x = fwd) */
#define SONAR_UY           {0, 1, -1, 0}     /* (y = left) */
#define SONAR_PERIOD_MS    30        /* all fire together -> full scan every 30 ms */
#define US_PER_CM          58
#define SONAR_MIN_CM       2         /* below this = invalid reading */
#define AVOID_CM           25        /* ~10 in reflex envelope */

/* ---------- IR beacon array (940 nm phototransistors, ADC1, brighter = higher) ---------- */
#define N_IR               5
#define IR_ADC_CH          {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5} /* GPIO1,2,4,5,6 */
#define IR_ANGLES          {60, 30, 0, -30, -60}  /* deg, + = left */
#define IR_DETECT          300       /* min peak count to count as "seen" */
#define IR_RANGE_SUM       2500      /* summed counts that mean "close enough to shoot" */
#define IR_AIM_TOL         4.0f      /* deg bearing error allowed when firing */

/* ---------- Dock (black floor) reflectance sensor, e.g. TCRT5000 ---------- */
#define DOCK_ADC_CH        ADC_CHANNEL_6   /* GPIO7 */
#define DOCK_BLACK         2800      /* counts above this = black dock */

/* ---------- Shooter: flywheel + feeder servo ---------- */
#define FLYWHEEL_PIN       41
#define SERVO_PIN          42
#define BALL_SW_PIN        8         /* ball-loaded switch, active low */
#define START_BTN_PIN      0         /* on-board BOOT button, active low */
#define FLYWHEEL_DUTY      900       /* of MOTOR_PWM_MAX */
#define SPINUP_MS          400
#define FEED_MS            250
#define SERVO_HZ           50
#define SERVO_HOLD_US      1000
#define SERVO_FIRE_US      2000

/* ---------- LEDC channel map ---------- */
#define LEDC_CH_FLY        4         /* motors use channels 0-3 */
#define LEDC_CH_SERVO      5

