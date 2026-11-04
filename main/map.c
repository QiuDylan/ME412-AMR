/* map.c - occupancy grid of the (known, rectangular) arena from the ToF ring
 *  - pose: dead-reckoned from commands + IMU yaw, corrected whenever a ray hits a wall
 *  - grid: log-odds; free along each ray, occupied at non-wall hits (obstacles)
 *  - planner: wavefront (BFS) from goal, robot steps toward the lowest neighbour */
#include <string.h>
#include "robot.h"

#define N_CELLS (MAP_NX * MAP_NY)
static int8_t grid[N_CELLS];
static uint16_t wave[N_CELLS];
static int16_t q[N_CELLS];
static float px = DOCK_X_MM, py = 0;
static const float ta[N_TOF] = TOF_ANGLES;

static int cell(float x, float y)
{
    int i = (int)(x / MAP_CELL_MM), j = (int)((y + ARENA_W_MM / 2) / MAP_CELL_MM);
    return (i >= 0 && i < MAP_NX && j >= 0 && j < MAP_NY) ? j * MAP_NX + i : -1;
}
static void bump(int k, int d) { if (k >= 0) { int v = grid[k] + d; grid[k] = (int8_t)(v > MAP_LO_MAX ? MAP_LO_MAX : v < -MAP_LO_MAX ? -MAP_LO_MAX : v); } }

void  map_reset(void) { px = DOCK_X_MM; py = 0; }
float map_bearing(float x, float y) { return atan2f(y - py, x - px); }

void map_update(const cmd_t *c)
{
    float yaw = align_yaw(), cy = cosf(yaw), sy = sinf(yaw), k = V_MAX_MMPS * LOOP_MS * 1e-3f;
    px += (c->vx * cy - c->vy * sy) * k; py += (c->vx * sy + c->vy * cy) * k;   /* dead-reckon */
    for (int i = 0; i < N_TOF; i++) {
        int r = tof_fresh(i); if (r < 0) continue;
        float a = yaw + ta[i], ca = cosf(a), sa = sinf(a), d = (float)r + TOF_R_MM;
        float tx = ca > 1e-3f ? (ARENA_L_MM - px) / ca : ca < -1e-3f ? -px / ca : 1e9f;            /* ray -> x walls */
        float ty = sa > 1e-3f ? (ARENA_W_MM / 2 - py) / sa : sa < -1e-3f ? (-ARENA_W_MM / 2 - py) / sa : 1e9f;
        float dw = fminf(tx, ty), end = fminf(d, TOF_MAX_MM);
        for (float s = TOF_R_MM; s < end - MAP_CELL_MM; s += MAP_CELL_MM) bump(cell(px + s * ca, py + s * sa), -MAP_FREE);
        if (fabsf(d - dw) < WALL_GATE_MM) { if (tx < ty) px += K_LOC * (dw - d) * ca; else py += K_LOC * (dw - d) * sa; }
        else if (d < dw && r < TOF_MAX_MM) bump(cell(px + d * ca, py + d * sa), MAP_HIT);
    }
    px = fminf(fmaxf(px, 0), ARENA_L_MM); py = fminf(fmaxf(py, -ARENA_W_MM / 2), ARENA_W_MM / 2);
}

static bool blocked(int i, int j)   /* occupied cell or any 8-neighbour (robot-size inflation) */
{
    for (int b = j - 1; b <= j + 1; b++) for (int a = i - 1; a <= i + 1; a++)
        if (a >= 0 && a < MAP_NX && b >= 0 && b < MAP_NY && grid[b * MAP_NX + a] > MAP_OCC) return true;
    return false;
}

static void plan(int goal)
{
    static const int8_t di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
    int h = 0, t = 0;
    memset(wave, 0xFF, sizeof wave); wave[goal] = 0; q[t++] = (int16_t)goal;
    while (h < t) {
        int k = q[h++], i = k % MAP_NX, j = k / MAP_NX;
        for (int n = 0; n < 4; n++) {
            int a = i + di[n], b = j + dj[n], m = b * MAP_NX + a;
            if (a < 0 || a >= MAP_NX || b < 0 || b >= MAP_NY || wave[m] != 0xFFFF || blocked(a, b)) continue;
            wave[m] = wave[k] + 1; q[t++] = (int16_t)m;
        }
    }
}

/* Drive toward (gx, gy) along the wavefront; hull turns to face travel direction while
 * mecanum translates immediately. Falls back to a straight line if unreachable. */
void map_goto(float gx, float gy, cmd_t *c)
{
    static int n, last_goal = -1;
    int g = cell(gx, gy), r = cell(px, py);
    if (g < 0 || r < 0) return;
    if (g != last_goal || ++n >= MAP_PLAN_EVERY) { plan(g); n = 0; last_goal = g; }
    float dir = map_bearing(gx, gy);
    int ri = r % MAP_NX, rj = r / MAP_NX, best = wave[r];
    for (int b = rj - 1; b <= rj + 1; b++) for (int a = ri - 1; a <= ri + 1; a++)
        if (a >= 0 && a < MAP_NX && b >= 0 && b < MAP_NY && wave[b * MAP_NX + a] < best) {
            best = wave[b * MAP_NX + a];
            dir = map_bearing((a + 0.5f) * MAP_CELL_MM, (b + 0.5f) * MAP_CELL_MM - ARENA_W_MM / 2);
        }
    float e = wrap_pi(dir - align_yaw());
    c->w = KP_YAW * e; c->vx = V_CRUISE * cosf(e); c->vy = V_CRUISE * sinf(e);
}

