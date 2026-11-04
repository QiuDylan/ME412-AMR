/* return_home.c - head back to own dock for reload after a shot
 * Plan through the map to the dock; when an IR source lines up with the map bearing to
 * home, steer on it directly. Done when the floor sensor sees the black dock. */
#include "robot.h"

void home_init(void) { adc_add(DOCK_ADC_CH); }   /* call after ir_init() */

bool dock_detected(void) { return adc_read(DOCK_ADC_CH) > DOCK_BLACK; }

bool home_cmd(const ir_t *ir, cmd_t *c)
{
    if (dock_detected()) { *c = (cmd_t){ 0 }; return true; }
    float b = ir->bearing;
    if (ir_seen(ir) && fabsf(wrap_pi(align_yaw() + b - map_bearing(0, 0))) < HOME_LOCK_RAD) {
        c->w = KP_YAW * b; c->vx = V_CRUISE * cosf(b); c->vy = V_CRUISE * sinf(b);   /* lock on home beacon */
    } else map_goto(DOCK_X_MM, 0, c);
    return false;
}
