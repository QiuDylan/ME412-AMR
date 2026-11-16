# CLAUDE.md — ME412-AMR

ESP-IDF firmware for an ME412 mecanum autonomous mobile robot (ESP32-S3).
Full mission code exists (`main.c` state machine: WAIT → LEAVE → HUNT → SHOOT → RETURN)
but the robot is currently running a **bench IR-tracking demo** (`IR_DEMO 1`).

## Build environment
- ESP-IDF **v5.4** at `/Users/pavankapoor/.espressif/v5.4/esp-idf` (installed via EIM CLI).
  CLI: `source ~/.espressif/tools/activate_idf_v5.4.sh && idf.py build`
- Target **esp32s3**. Board: Waveshare ESP32-S3 DevKit **N16R8** (16MB flash, 8MB octal PSRAM).
- The user builds/flashes/monitors from the ESP-IDF VS Code extension. **Don't flash.**
  To verify compiles without touching the repo, build out-of-tree:
  `idf.py -B <scratch>/build -D SDKCONFIG=<scratch>/sdkconfig build`
- System `python3` is 3.15 (too new for ESP-IDF); Homebrew `python3.12` exists. The
  installed IDF has its own venv, so this only matters when reinstalling.
- No `.gitignore` yet: `build/` and `sdkconfig` will appear untracked after a build.
- Work is **uncommitted** on `main` (new: `yahboom.c`, `ir_demo.c`, this file; modified:
  `config.h`, `main.c`, `robot.h`, `main/CMakeLists.txt`). Commit only when asked.

## Hardware actually on the robot now
- **Motors:** 4× JGY370 12V 100 rpm worm-gear, 11-line Hall encoders, 1:60, 97 mm mecanum wheels.
- **Motor driver:** Yahboom 4-channel encoder motor driver (STM32 co-processor), **I2C 0x26**,
  12V battery into the driver. The L298N code in `drive.c` is **not** the current hardware.
- **Yahboom I2C:** SDA **GPIO 10**, SCL **GPIO 11**, own bus `I2C_NUM_1`, 400 kHz.
  (These pins collide with `MOTOR_EN_PINS`/`TOF_XSHUT_PINS` in config.h — harmless while
  IR_DEMO=1 because drive/avoid aren't initialised. User said pins may change later; keep for now.)
- **IR sensor:** one BPV11F phototransistor on **GPIO 5** (ADC1_CH4).
- **IMU:** MPU-6050 (GY-521) on the **Yahboom bus** (GPIO 10/11) at **0x68**, `main/mpu6050.c`.
  Gyro-Z integrated to heading (no magnetometer → slow drift). Mounted near the motors for now
  (user will move it). Not yet verified on the robot (sign, drift).
- **START button:** GPIO 0 = on-board BOOT button. Holding it during reset enters download mode.
- **Not yet fitted/used in demo:** BNO080 IMU (planned; may replace the MPU-6050),
  VL53L1X ToF ring, CO2 valve servo, dock sensor. Those use bus `I2C_NUM_0` on GPIO 41/42.
- ESP32 currently USB-powered (tethered).

## Yahboom driver facts (verified on the robot)
- Registers: 0x01 type, 0x02 deadzone (0–3600), 0x03 encoder lines, 0x04 ratio (uint16),
  0x05 wheel dia (float LE), 0x06 speed 4×int16 BE (−1000..1000), 0x07 PWM 4×int16 BE,
  0x10–0x13 per-10ms counts, 0x20–0x27 total counts (hi/lo word per motor).
- Config sent at init: type 1, ratio 60, lines 11, wheel 97.0, deadzone 1600 (deadzone untuned).
- **Motor positions:** M1 = rear-right, M2 = front-right, M3 = rear-left, M4 = front-left
  → `YB_WHEEL_MAP {3, 1, 2, 0}` (index order FL, FR, RL, RR).
- `YB_DIR {-1,-1,-1,-1}`: every wheel needed inverting so + speed = robot forward (wheel test).
- `YB_ENC_SIGN -1`: the driver's encoder counts run opposite to its speed sign.
- **No command timeout on the driver**: it keeps the last speed across ESP32 resets/hangs. So `yb_init`
  sends 0 first (and runs before WiFi), and a watchdog task zeroes the wheels if a non-zero command isn't
  refreshed within `YB_WATCHDOG_MS` (300) → anything that drives must resend every loop.
- Must use the **new `i2c_master` API** — mixing legacy `driver/i2c.h` aborts at boot in IDF 5.4.

## IR demo (`main/ir_demo.c`, settings `IRD_*` in config.h)
- One narrow-angle sensor → find the source by rotating and remembering the brightest angle.
  Heading = wheel-encoder rotation: `rot = (-FL + FR - RL + RR) / 4`, + = CCW.
- **Behaviour (user's spec, 2026-11-10):** START → spin up to 360°. When signal ≥ ambient + `IRD_DETECT`,
  keep turning until it drops below `IRD_PAST_PEAK` × peak (or `IRD_PAST_DEG` past it), brake, then
  yaw-control `face()` back to the peak heading → LOCKED (sits still). Re-spin **only** if the source is
  unseen for `IRD_LOST_MS` (1 s). A spin that finds nothing → wait `IRD_SEARCH_PERIOD_MS` (10 s), spin again.
  Source appearing while waiting → lock without moving. (Old ±40° re-centre sweep removed.)
- `IRD_USE_IMU 1`: heading from the MPU-6050, scaled into the same "counts" so the demo logic is
  unchanged (falls back to encoders if no IMU). `IRD_SENSE_TEST 1`: no tracking, prints IR + IMU/encoder
  heading at 5 Hz; START/Enter = one 360° CCW turn by gyro via `sweep()`, prints overshoot + IR peak angle.
- **WiFi monitor** (`main/netlog.c`, `NET_LOG 1`): ESP32 runs its own AP "ME412-AMR" (pass in
  config.h), TCP server on 3333 → laptop `nc 192.168.4.1 3333`. robot.h maps `printf` → `net_printf`
  (USB + TCP). In the client: Enter = START, `x`+Enter = latch wheels off until reset. Not yet tested.
- Modes in config.h: `IR_DEMO` (1 = demo instead of mission), `IRD_CAL` (measure counts per turn
  with START presses), `YB_WHEEL_TEST` (run each wheel forward in turn).
- **Calibrated:** `IRD_COUNTS_PER_360 = 6817` (one run on the bench floor).

## Status
- Verified on robot: Yahboom comms, wheel map/directions, encoder sign, in-place spin is correct.
- 2026-11-10: IR detection checked stationary (`IRD_SENSE_TEST 1`, `MOTORS_OFF 1`): works reasonably
  well with `IRD_DETECT` = 70 (lowered from 300 → 150 → 70). Motors re-enabled afterwards (`MOTORS_OFF 0`).
- 2026-11-10 untethered (WiFi): motors wouldn't stop and the feed went silent once IR was seen. Cause
  not yet known (LED/RMT block, brownout, or a stalled WiFi feed). Added: watchdog, non-blocking LED,
  reset reason in the boot log + `nc` banner, sense-test spin auto-stops after 360° (gyro) / 20 s.
- 2026-11-10: **M3 (rear-left) runs backwards on its own** from 12V power-on, even with speed 0 sent.
  Open-loop PWM 0 (`yb_brake()`, reg 0x07) stops it → the driver's speed loop is the cause, most likely
  a bad M3 encoder signal (loose/broken/swapped wire). Sense test idles on `yb_brake()`. After one PWM-0 stop, M3 behaved normally again (spin
  correct) - likely wound-up PID state that survived a 12V cycle. `yb_init` now sends `yb_brake()` first.
  If M3 runs away again, check its encoder wiring.
- 2026-11-10: 360° gyro turn stopped at 408° on the IMU, and physically ~408° too → gyro scale is good;
  overshoot came from ending turns with speed 0 (driver PID ramps down). `sweep()` now ends with
  `yb_brake()` (PWM 0) and records the heading at the stop command (`scan_t.end`). With brake: 403°. Then added lead compensation in
  `sweep()`: stop when remaining ≤ rate × `coast_s` (start `IRD_COAST_S` 0.4 s), learn `coast_s` from each
  turn's measured overshoot (resets on reboot). Prints `IRD: turn end overshoot ...` per turn. First turn
  with it: 341° (over-corrected). Then added **yaw control** `face(target)`: P-control on gyro heading
  after the coarse turn, brake inside ±`IRD_TOL_DEG` (2°), done after 100 ms inside, gives up after
  `IRD_HOLD_MS`. Used by `point_at()` (demo locks) and the sense-test 360. Re-test pending.
- 2026-11-10: ESP32 froze/crashed the moment IR was first seen (twice, both with WiFi on); `nc` showed
  nothing on reconnect. Suspects: RGB LED (RMT) blink that starts on detection, or main-task stack
  (3.5 KB) overflow. Demo now runs in its own 8 KB task (main.c) and `IRD_LED 0` disables the blink.
- **Not yet verified:** the full IR search/lock/re-centre/lost behaviour, `IRD_DETECT` threshold
  vs real ambient, turn accuracy/overshoot (`IRD_MIN_SPEED`, `IRD_KP`), deadzone tuning.
- Next likely work: tune the demo from monitor output; then port `drive.c` to the Yahboom
  driver (keep the mecanum kinematics, `drive_set` → `yb_wheels`), resolve pin conflicts,
  encoder odometry for `map.c`, add the IMU.

## Known issues in the original code (from review, not yet fixed)
- `TRACK_TEST 1` (mission bench mode); no init error checking in `main.c`;
  RGB LED may be GPIO 48 on some DevKits (code uses 38; 48 is also a ToF XSHUT);
  no `sdkconfig.defaults` (builds as 2MB flash, PSRAM off); README empty.

## Working with this user
- Asked to see diffs before changes in earlier work — show a diff for non-trivial edits.
- Give step-by-step instructions for VS Code / ESP-IDF actions (newer to the toolchain).
