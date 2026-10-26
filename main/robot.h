/* robot.h - shared types + module interfaces */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "esp_timer.h"
#include "config.h"

typedef struct { float vx, vy, w; } cmd_t;          /* fwd, left strafe, CCW (-1..1) */
typedef struct { int sum, peak; float bearing; } ir_t; /* bearing deg, + = left */

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

/* drive.c */
void drive_init(void);
void drive_set(float vx, float vy, float w);

/* obstacle.c */
void obstacle_init(void);
bool avoid_cmd(cmd_t *c);                 /* overrides translation if obstacle near */

/* ir_track.c */
void ir_init(void);
void ir_read(ir_t *r);
bool track_cmd(const ir_t *r, cmd_t *c);  /* false if no beacon seen */
bool ir_aligned(const ir_t *r);
bool dock_detected(void);

/* shooter.c */
void shooter_init(void);
void shooter_fire(void);
bool shooter_busy(void);                  /* call every loop; advances shot sequence */
bool ball_loaded(void);
