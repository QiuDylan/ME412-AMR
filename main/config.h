/* config.h - ALL pinouts and tuning constants (Waveshare ESP32-S3)
 * Avoid: GPIO0/45 (strapping), 19/20 (USB), 26-37 (flash/PSRAM), 43/44 (UART0).
 * Verify every pin against your specific Waveshare board's pinout. */
#pragma once
#include "esp_adc/adc_oneshot.h"

#define DEG(x)             ((x) * 0.01745329f)   /* degrees -> radians (compile-time) */

/* ---------- Timing ---------- */
#define LOOP_MS            20        /* main control loop period (50 Hz) */
#define LEAVE_DOCK_MS      1500      /* blind drive-out time after start */
#define RUN_TIMEOUT_MS     60000     /* give up hunting and go home after this */

/* ---------- Arena / world frame: origin = own home base, +x toward opponent, +y left ---------- */
#define ARENA_L_MM         3048      /* 10 ft */
#define ARENA_W_MM         1829      /* 6 ft  */
#define DOCK_X_MM          200       /* start / dock pose (robot centre) */
#define OPP_X_MM           (ARENA_L_MM - 600)   /* hunt goal: in front of opponent base */

/* ---------- Shared I2C bus: BNO080 + VL53L1X ring ---------- */
#define I2C_SDA_PIN        41
#define I2C_SCL_PIN        42
#define I2C_HZ             400000

/* ================= drive.c : mecanum on 2x L298N (ENx = PWM, INx = direction) ================= */
/* wheel order FL, FR, RL, RR. Remove the ENA/ENB jumpers on the L298N boards. */
#define MOTOR_EN_PINS      {11, 14, 17, 38}
#define MOTOR_IN1_PINS     {12, 15, 18, 39}
#define MOTOR_IN2_PINS     {13, 16, 21, 40}
#define MOTOR_INVERT       {0, 1, 0, 1}  /* flip a wheel's direction without rewiring */
#define MOTOR_PWM_HZ       5000      /* L298N is slow (bipolar); keep PWM <= ~5-10 kHz */
#define MOTOR_PWM_BITS     LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX      1023
#define V_MAX_MMPS         600.0f    /* ground speed at command 1.0 (calibrate; used for dead-reckoning) */
#define V_CRUISE           0.6f      /* normalized speeds (0..1) */
#define V_APPROACH         0.4f
#define V_AVOID            0.7f

/* ================= avoid.c : VL53L1X time-of-flight ring ================= */
enum { TOF_F, TOF_FL, TOF_FR, TOF_L, TOF_R, TOF_B, N_TOF };
#define TOF_ANGLES         {0, DEG(45), DEG(-45), DEG(90), DEG(-90), DEG(180)}  /* rad, + = left */
#define TOF_XSHUT_PINS     {-1, 9, 10, 3, 48, 46} /* -1: no XSHUT (index 0 only). GPIO46 is a boot strap:
                                                     OK for normal boot, don't hold BOOT while XSHUT pulled up */
#define TOF_ADDR0          0x30      /* sensor i is re-addressed to TOF_ADDR0 + i */
#define TOF_TB_A           0x0060    /* timing budget 33 ms, long mode (ST ULD values; verify) */
#define TOF_TB_B           0x006E
#define TOF_IM_MS          35        /* inter-measurement period, >= timing budget */
#define TOF_POLL_MS        10
#define TOF_R_MM           60        /* sensor distance from robot centre */
/* --- Distance thresholds (mm) : TUNE ON ARENA --- */
#define TOF_MAX_MM         3500      /* no target / invalid -> treat as this far */
#define AVOID_MM           {250, 200, 200, 150, 150, 150}  /* per-sensor repulsion envelope */
#define SLOW_MM            450       /* front: start scaling forward speed down */
#define STOP_MM            100       /* front: zero forward speed, back off below this */
#define SIDESTEP           0.5f      /* strafe bias toward open side when front blocked */

/* ================= map.c : occupancy grid + wall localisation + wavefront planner ================= */
#define MAP_CELL_MM        100
#define MAP_NX             ((ARENA_L_MM + MAP_CELL_MM - 1) / MAP_CELL_MM)   /* 31 */
#define MAP_NY             ((ARENA_W_MM + MAP_CELL_MM - 1) / MAP_CELL_MM)   /* 19 */
#define MAP_HIT            20        /* log-odds added for a hit */
#define MAP_FREE           4         /* log-odds removed for a pass-through */
#define MAP_LO_MAX         100       /* log-odds clamp */
#define MAP_OCC            40        /* log-odds above this = occupied */
#define WALL_GATE_MM       80        /* reading within this of predicted wall = wall hit */
#define K_LOC              0.3f      /* pose correction gain per wall hit */
#define MAP_PLAN_EVERY     5         /* replan wavefront every N control loops */

/* ================= ir_tracker.c : BPV11F phototransistor ring, 360 deg (ADC1) ================= */
/* wire: collector -> 3V3, emitter -> ADC + resistor to GND (brighter = higher counts) */
#define N_IR               6
#define IR_ADC_CH          {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6} /* GPIO1,2,4,5,6,7 */
#define IR_ANGLES          {0, DEG(60), DEG(120), DEG(180), DEG(-120), DEG(-60)}  /* rad, + = left */
#define IR_DETECT          300       /* min peak count to count as "seen" */
#define IR_RANGE_SUM       2500      /* summed counts that mean "close enough to shoot" */
#define KP_TRACK           0.7f      /* w per rad of IR bearing */

/* ================= align.c : BNO080 IMU (SHTP over I2C) ================= */
#define IMU_ADDR           0x4B      /* 0x4A if SA0 pulled low */
#define IMU_PERIOD_MS      5         /* poll / report rate 200 Hz */
#define IMU_RD_LEN         32        /* one read holds timestamp + game-rotation + gyro reports */
#define IMU_YAW_SIGN       1.0f      /* set -1 if yaw grows clockwise (IMU mounted upside down) */
#define HOME_EXCL_RAD      DEG(45)   /* never fire within +-45 deg of bearing to own base */

/* ================= shoot.c : servo-actuated CO2 valve, hull aim via IMU ================= */
#define VALVE_SERVO_PIN    47
#define SERVO_HZ           50
#define VALVE_CLOSED_US    1000
#define VALVE_OPEN_US      2000
#define VALVE_OPEN_MS      150       /* CO2 burst length */
#define KP_AIM             1.2f      /* w per rad heading error */
#define KD_AIM             0.08f     /* w per rad/s yaw rate (damping) */
#define AIM_TOL_RAD        DEG(2)    /* heading error allowed when firing */
#define AIM_RATE_TOL       DEG(10)   /* rad/s, hull must be settled before firing */
#define AIM_IR_BLEND       0.2f      /* fraction of each fresh IR fix used to refine target heading */
#define AIM_TIMEOUT_MS     2000
#define LEDC_CH_VALVE      4         /* motors use LEDC channels 0-3 */

/* ================= return_home.c ================= */
#define DOCK_ADC_CH        ADC_CHANNEL_7   /* GPIO8, TCRT5000 floor sensor */
#define DOCK_BLACK         2800      /* counts above this = black dock */
#define HOME_LOCK_RAD      DEG(20)   /* IR within this of map bearing to home = home beacon */
#define KP_YAW             0.8f      /* w per rad of heading error */

/* ================= main.c ================= */
#define START_BTN_PIN      0         /* on-board BOOT button, active low */
