/* robot.h - shared types + module interfaces (see block diagram) */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "config.h"

typedef struct { float vx, vy, w; } cmd_t;             /* fwd, left strafe, CCW (-1..1) */
typedef struct { int sum, peak; float bearing; } ir_t;  /* bearing rad, body frame, + = left */

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static inline float wrap_pi(float a) { return atan2f(sinf(a), cosf(a)); }

/* drive.c */
void drive_init(void);
void drive_set(float vx, float vy, float w);

/* avoid.c - VL53L1X ring -> reflexive avoidance */
void avoid_init(i2c_master_bus_handle_t bus);
int  tof_fresh(int i);                       /* new range (mm) since last call, else -1 */
bool avoid_cmd(cmd_t *c);                    /* overrides translation if obstacle near */

/* map.c - occupancy grid, wall-based localisation, wavefront path planning */
void  map_reset(void);                       /* pose -> dock, keeps grid */
void  map_update(const cmd_t *c);            /* call once per loop with last command */
void  map_goto(float gx, float gy, cmd_t *c);
float map_bearing(float x, float y);         /* world-frame bearing from robot to (x,y) */

/* ir_tracker.c - opponent base / opponent beacon */
void ir_init(void);
void ir_read(ir_t *r);
bool ir_seen(const ir_t *r);
bool ir_in_range(const ir_t *r);
bool track_cmd(const ir_t *r, cmd_t *c);     /* false if no (safe) beacon seen */
void adc_add(adc_channel_t ch);              /* shared ADC1 unit */
int  adc_read(adc_channel_t ch);

/* align.c - BNO080 heading, yaw 0 = +x (facing opponent) */
void  align_init(i2c_master_bus_handle_t bus);
void  align_zero(void);
float align_yaw(void);                       /* rad, CCW +, wrapped to +-pi */
float align_rate(void);                      /* rad/s */
bool  align_safe(float heading);             /* false if heading points at own base */

/* shoot.c - hull aim + CO2 valve */
void shoot_init(void);
void shoot_start(float heading);             /* lock target heading */
int  shoot_update(const ir_t *ir, cmd_t *c); /* 1 busy, 0 fired, -1 aborted */

/* return_home.c */
void home_init(void);
bool dock_detected(void);
bool home_cmd(const ir_t *ir, cmd_t *c);     /* true once docked */
