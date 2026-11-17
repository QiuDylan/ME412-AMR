# Handoff: ToF obstacle avoidance (ME412-AMR)

For an agent picking up "use the ToF sensor to avoid obstacles". Read `CLAUDE.md` first: it has the
build setup, the Yahboom motor-driver facts and how this user likes to work. This file covers only
the time-of-flight (ToF) work.

Starting point: branch `IR_Demo_works`, commit `4ad2780 "TOF"` (pushed). Last change to this file: 2026-11-17.

## What works (verified on the robot)

- **One VL53L0X ranges correctly.** It is wired **directly** to its own I2C bus, not through the mux:
  - SDA **GPIO 13**, SCL **GPIO 14**, `I2C_NUM_0`, 100 kHz, address **0x29**.
  - The sensor board has its own pull-up resistors.
- The board is a generic **"VL53L0/1XV2"** breakout. That design ships with either a VL53L0X or a
  VL53L1X. This one reads model ID **0xEE**, so it is a **VL53L0X**.
- Boot test: `TOF_TEST 1` in `config.h`. It runs `tof_test_main()` in `main/vl53l0x.c` instead of the
  IR demo, and prints `TOF direct0: <mm> (status N)` at 5 Hz to USB and the WiFi log
  (`nc 192.168.4.1 3333`).
  - Status **11** is a valid reading.
  - **8190/8191 mm** with status 4 or 6 means no target (signal or phase fail).
  - Range is about 1.2 m on good targets; it is shorter on dark or shiny ones.
  - The cone is narrow, about 25°.

## What does not work / is uncertain

- **The TCA9548A mux never worked reliably.** It answered only once, at 0x76.
  - That board has **no pull-ups on RST or A0–A2**: RST must be wired to 3V3 and A0–A2 to GND.
  - Even with those wired and measured correctly, it stopped answering.
  - It is probably damaged. The user has since taken it out of the circuit.
  - The test code still supports it (searches 0x70–0x77, re-searches if it drops out, uses
    `TOF_MUX_CH`) and falls back to talking to the sensor directly when no mux answers.
  - With more than one sensor, every sensor is at 0x29, so you need either a working mux or XSHUT
    pins to give each sensor a new address. Ask the user which.
- **The setup skips the "measurement timing budget" step from Pololu's driver.** The sensor uses its
  default of about 30 ms per reading. Readings are fine on the bench, but accuracy has not been
  checked against a ruler.
- **This clone chip once stopped answering I2C during calibration.** `wait_bit()` now tolerates
  refused reads while it waits and counts them. Keep that behaviour.
- **Chip detection writes to a VL53L0X register.** `sensor_start()` first tries a VL53L1X ID read,
  which on a VL53L0X writes register 0x01 = 0x0F. That is harmless because setup rewrites 0x01, but
  remember it if you reorder the code.

## Code map

| File | Status |
|---|---|
| `main/vl53l0x.c` | **Working VL53L0X driver** (Pololu port) plus VL53L1X detection and setup. Everything is `static`; only `tof_test_main()` is visible to other files. Readings block for up to 200 ms. |
| `main/avoid.c` | Original mission code for a **6× VL53L1X** ring using XSHUT and new addresses. **Wrong chip and wiring for this robot.** Its `avoid_cmd(cmd_t*)` logic (slow down / stop / push away from close sensors / sidestep) is reusable. It is only called from the mission path in `main.c`. |
| `main/map.c` | Uses `tof_fresh(i)` from avoid.c to update the occupancy map. Mission code only. |
| `main/drive.c` | **L298N driver: not the current hardware.** The mecanum mixing is correct and reusable. |
| `main/yahboom.c` | The real motor driver (see below). |
| `main/config.h` | ToF block: `TOF_TEST`, `TOF_USE_YB_BUS` (0 = own bus on 13/14), `TOF_SDA_PIN`, `TOF_SCL_PIN`, `TOF_I2C_PORT`, `TOF_I2C_HZ`, `MUX_ADDR`, `VL53L0X_ADDR`, `TOF_MUX_CH`. Avoidance thresholds already exist: `SLOW_MM 450`, `STOP_MM 100`, `AVOID_MM`, `V_AVOID`, `SIDESTEP`. |

## Driving the robot (needed for avoidance)

- **Wheel commands:** `yb_wheels(int16_t w[4])`. Order is FL, FR, RL, RR; range −1000..1000;
  + = drives the robot forward.
- **The command must be re-sent at least every 300 ms.** A watchdog zeroes the wheels if a non-zero
  command isn't refreshed within `YB_WATCHDOG_MS`. Send every loop.
- **Stopping:**
  - `yb_brake()` is a hard stop: it sends speed 0, then PWM 0.
  - `yb_spin(s)` turns in place; + = CCW.
- **Mecanum mixing** from `drive.c`, with vx = forward, vy = strafe left, w = CCW:
  `FL = vx - vy - w`, `FR = vx + vy + w`, `RL = vx + vy - w`, `RR = vx - vy + w`.
  - The w terms match `yb_spin`, so turning is verified.
  - **The strafe sign has never been tested on this robot.**
- **Remote stop:** typing `x` + Enter in the `nc` client makes `yb_wheels` send 0 until reset.
- **Bench safety:** `MOTORS_OFF 1` forces every wheel command to 0.
- **Heading:** `mpu_init(yb_bus())` / `mpu_yaw_deg()` gives gyro heading from the MPU-6050 on the
  motor bus. It drifts slowly.

## Suggested approach

1. **Turn `vl53l0x.c` into a module.**
   - Add `tof_start()` to set up the bus and sensor once, and a background task that reads in
     continuous mode and caches the latest mm (invalid readings become `TOF_MAX_MM`).
   - Add non-blocking `int tof_mm(void)` (and `tof_fresh` if `map.c` is ever revived).
   - Keep `tof_test_main()` as the bench test.
   - The control loop must never wait on I2C: the Yahboom watchdog needs a command every loop.
2. **Add a mode such as `AVOID_DEMO`** next to `IR_DEMO` / `TOF_TEST`:
   - Drive forward slowly.
   - Scale speed down between `SLOW_MM` and `STOP_MM`.
   - Below `STOP_MM`: brake, back off briefly, turn in place (by gyro) until the front reading is
     clear, then continue.
   - Lower `TOF_MAX_MM` (3500 is for the VL53L1X) to about 1200–2000 for the VL53L0X.
3. **Reuse the logic in `avoid_cmd()`**, but feed it the single front sensor. The side and back
   entries can stay at `TOF_MAX_MM` until more sensors are fitted.
4. **Start every test with `MOTORS_OFF 1`** and watch the WiFi log, then enable the motors.

## Known traps

- `config.h`: the mission bus `I2C_SDA_PIN`/`I2C_SCL_PIN` = 41/42 **also uses `I2C_NUM_0`**. If the
  mission code (`IR_DEMO 0`, `TOF_TEST 0`) and the ToF bus are ever initialised together, they clash.
  Pick one bus for all sensors.
- GPIO 13/14 also appear in the L298N `MOTOR_IN*` pins, and GPIO 10 in `TOF_XSHUT_PINS`. Both are
  harmless while the L298N/avoid.c code isn't initialised; clean them up when touching that code.
- `TOF_TEST 1` takes priority over `IR_DEMO 1` in `main.c`. Set `TOF_TEST 0` to get the IR demo back.
- **The `build/` directory is committed to git** (it was committed before `.gitignore` existed).
  Builds dirty the working tree. Don't commit build output; mention it to the user.

## Working with this user

- They flash and monitor from the ESP-IDF VS Code extension. **Don't flash.**
- Check compiles out-of-tree: `idf.py -B <scratch>/build -D SDKCONFIG=<scratch>/sdkconfig build`
  (after `source ~/.espressif/tools/activate_idf_v5.4.sh`).
- Show a diff for non-trivial edits.
- Give step-by-step VS Code instructions; they're newer to the toolchain. In VS Code, use
  Cmd+Shift+P → "ESP-IDF: Build, Flash and Start a Monitor on your Device", and select the
  serial port if flashing fails.
- Commit or push only when asked.
- They debug by pasting monitor screenshots, so print clear, labelled one-line status messages.
