/* return_home.c - head back to own dock for reload after a shot
 * Steer on IMU heading toward home (yaw = pi); once roughly facing home, steer on the
 * home-base IR instead. Done when the floor sensor sees the black dock. */
#include "robot.h"

void home_init(void) { adc_add(DOCK_ADC_CH); }   /* call after ir_init() */

bool dock_detected(void) { return adc_read(DOCK_ADC_CH) > DOCK_BLACK; }

bool home_cmd(const ir_t *ir, cmd_t *c)
{
    if (dock_detected()) { *c = (cmd_t){ 0 }; return true; }
    float err = wrap_pi((float)M_PI - align_yaw());
    if (fabsf(err) < HOME_LOCK_RAD && ir_seen(ir)) err = ir->bearing;   /* lock on home beacon */
    c->w = KP_YAW * err; c->vx = V_CRUISE * fmaxf(0, cosf(err)); c->vy = 0;   /* slow while turning */
    return false;
}

