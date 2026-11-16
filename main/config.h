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
#define MOTOR_EN_PINS      {11, 14, 17, 45}  /* GPIO45 = VDD_SPI strap: OK as EN (internal pulldown at
                                                boot), never put a pull-up on it. GPIO38 is the RGB LED. */
#define MOTOR_IN1_PINS     {12, 15, 18, 39}
#define MOTOR_IN2_PINS     {13, 16, 21, 40}
#define MOTOR_INVERT       {0, 1, 0, 1}  /* flip a wheel's direction without rewiring */
#define MOTOR_PWM_HZ       5000      /* L298N is slow (bipolar); keep PWM <= ~5-10 kHz */
#define MOTOR_PWM_BITS     LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX      1023
#define MOTOR_MIN_DUTY     0.30f     /* L298N + motor stall floor: any nonzero command maps to >= this duty (TUNE) */
#define DRIVE_DEADZONE     0.03f     /* normalized wheel command below this = wheel off */
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
#define IR_OVERSAMPLE      4         /* ADC reads averaged per channel per loop */
#define IR_DETECT          300       /* min peak (ambient-subtracted counts) to count as "seen" */
#define IR_RELEASE_FRAC    0.6f      /* hysteresis: stays "seen" until peak < IR_DETECT * this */
#define IR_RANGE_SUM       2500      /* summed (ambient-subtracted) counts = "close enough to shoot" */
#define IR_SMOOTH          0.5f      /* low-pass on bearing vector, 1 = none, smaller = smoother/slower */
#define KP_TRACK           0.7f      /* w per rad of IR bearing */
#define IR_ALIGN_TOL       DEG(5)    /* bearing error treated as "facing the beacon": stop rotating */
#define IR_W_MAX           0.6f      /* max rotation command while tracking */
#define IR_SEARCH_W        0.35f     /* in-place search spin when beacon is lost */
#define IR_LOST_MS         300       /* beacon unseen this long -> start search spin */
#define TRACK_TEST         1         /* 1 = bench test: after START only sense IR + rotate to face it */
/* --- IR detect indicator (on-board WS2812 RGB LED) --- */
#define RGB_LED_PIN        38        /* on-board addressable RGB LED (ESP32-S3 DevKit) */
#define IR_LED_THRESH      200       /* any IR sensor above this = blink LED (tune vs ambient) */
#define IR_BLINK_MS        100       /* 100 ms on / 100 ms off */
#define IR_DEBUG           1         /* 1 = print IR readings (set 0 for competition) */
#define IR_DEBUG_EVERY     5         /* print every N loops (5 = 10 Hz) */
 
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

/* ================= yahboom.c : Yahboom 4-ch encoder motor driver (I2C 0x26) ================= */
/* Own I2C bus, separate from the BNO080/ToF bus. NOTE: GPIO10/11 are also used above (TOF_XSHUT /
 * MOTOR_EN). Fine while IR_DEMO = 1 (those modules aren't initialised) - resolve before merging. */
#define YB_I2C_PORT        I2C_NUM_1
#define YB_SDA_PIN         10
#define YB_SCL_PIN         11
#define YB_ADDR            0x26
#define YB_HZ              400000
#define YB_TIMEOUT_MS      20
#define YB_MOTOR_TYPE      1         /* driver type 1 = 520-style 11-line Hall encoder motor */
#define YB_GEAR_RATIO      60        /* JGY370 100 rpm, 1:60 (integer only) */
#define YB_ENC_LINES       11
#define YB_WHEEL_MM        97.0f     /* mecanum wheel diameter */
#define YB_WATCHDOG_MS     300       /* zero the wheels if a non-zero command isn't refreshed this often */
#define YB_DEADZONE        1600      /* TODO: tune (0-3600) */
#define YB_WHEEL_MAP       {3, 1, 2, 0}      /* driver motor (M1..M4 = 0..3) for FL, FR, RL, RR:
                                                M1 = RR, M2 = FR, M3 = RL, M4 = FL (wheel-tested) */
#define YB_DIR             {-1, -1, -1, -1}  /* so + speed = wheel drives robot forward (bench-tested) */
#define YB_ENC_SIGN        -1        /* driver's encoder counts run opposite to its speed sign (CAL read -6817) */

/* ================= mpu6050.c : MPU-6050 gyro heading (shares the Yahboom I2C bus) ================= */
/* wire: VCC 3V3, GND, SDA GPIO10, SCL GPIO11, AD0 -> GND (0x68). Mount flat, chip Z axis up. */
#define MPU_ADDR           0x68      /* 0x69 if AD0 is tied high */
#define MPU_YAW_SIGN       1.0f      /* -1 if the board is mounted upside down (yaw grows clockwise) */
#define MPU_DLPF           4         /* on-chip low-pass: 4 = ~20 Hz (rejects motor vibration) */
#define MPU_BIAS_SAMPLES   200       /* gyro offset average at boot, 10 ms each -> keep still ~2 s */
#define MPU_DEADBAND_DPS   0.3f      /* rates below this (after offset) count as zero: less drift at rest */

/* ================= netlog.c : WiFi serial monitor (robot runs its own network) ================= */
#define NET_LOG            1         /* 1 = WiFi log + remote START/STOP; 0 = USB serial only */
#define NET_SSID           "ME412-AMR"
#define NET_PASS           "me412robot"   /* WPA2, min 8 characters */
#define NET_CHANNEL        6
#define NET_PORT           3333      /* laptop: nc 192.168.4.1 3333 */

/* ================= ir_demo.c : single BPV11F IR tracking demo ================= */
#define IR_DEMO            1         /* 1 = boot into the IR demo (skips L298N/ToF/IMU/mission code) */
#define MOTORS_OFF         0         /* 1 = every wheel command is sent as 0 (bench: robot stays put) */
#define IRD_SENSE_TEST     0         /* 1 = sensor check: print IR + headings; START / Enter = one 360 turn by gyro */
#define IRD_USE_IMU        1         /* 1 = heading from the MPU-6050 gyro (falls back to encoders if missing) */
#define IRD_CAL            0         /* 1 = measure IRD_COUNTS_PER_360 instead (see ir_demo.c header) */
#define YB_WHEEL_TEST      0         /* 1 = on START, run FL, FR, RL, RR forward one at a time (fix MAP/DIR) */
#define IRD_ADC_CH         ADC_CHANNEL_4   /* GPIO5 */
#define IRD_OVERSAMPLE     8
#define IRD_LED            0         /* 1 = blink the RGB LED while IR is seen (off: suspected freeze on detect) */
#define IRD_DETECT         70        /* counts above ambient = source seen (was 300; lowered after bench tests) */
#define IRD_SEARCH_PERIOD_MS 10000   /* a spin that found nothing: spin again after this */
#define IRD_RELEASE        0.6f      /* locked: still counts as seen down to this x IRD_DETECT */
#define IRD_LOST_MS        1000      /* locked: source unseen this long -> spin again */
#define IRD_PAST_PEAK      0.7f      /* after detecting, stop once signal < this x peak (past the beam centre) */
#define IRD_PAST_DEG       30.0f     /* ...or this far past the peak, whichever first */
#define IRD_COUNTS_PER_360 6817      /* measured with IRD_CAL = 1 (one run; re-check on the arena floor) */
#define IRD_SEARCH_SPEED   150       /* Yahboom speed units (-1000..1000) while turning */
#define IRD_MIN_SPEED      60        /* slowest turn that still moves (TUNE with YB_DEADZONE) */
#define IRD_COAST_S        0.4f      /* start guess: s of turning after the stop is sent (learned each turn) */
#define IRD_TOL_DEG        2.0f      /* yaw control: done when within +- this */
#define IRD_HOLD_KP        0.4f      /* yaw control speed per count of error (19 counts/deg) */
#define IRD_HOLD_MAX       100       /* yaw control max turn speed (slower than the scan) */
#define IRD_HOLD_MS        3000      /* yaw control gives up after this */
#define IRD_KP             0.5f      /* turn speed per remaining count (slow-down near target) */
#define IRD_SWEEP_TIMEOUT_MS 20000   /* abort a turn that isn't finishing (stall / I2C errors) */
#define IRD_DEBUG_EVERY    10        /* status print every N idle loops (10 = 5 Hz) */
