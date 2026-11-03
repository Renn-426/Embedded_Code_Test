# INF2004 robot car - handoff (2026-11-03)

Repo: github.com/ayumisit/INF2004-EMBEDDED-SYSTEMS
Branch: `fix/robot-movement-barcode-mqtt`, base commit `7aa34de`
(Merge PR #4 from buddy5). **Nothing below is committed.** Each buddy should
commit their own part (see "Split" at the end).

## What's in this zip

| Path | What |
|---|---|
| `source/` | Full project tree as it runs on the robot (base + all changes). Wi-Fi credentials removed. |
| `working_changes.patch` | Just the changes, 30 modified + 2 new files. Applies cleanly on `7aa34de`. |
| `HANDOFF.md` | This file. |

Apply the patch to your own clone:

    git checkout fix/robot-movement-barcode-mqtt   # at 7aa34de
    git apply working_changes.patch

## Setup before building

1. `cp config/wifi_credentials.example.h config/wifi_credentials.h` and fill in
   SSID, password and `MQTT_BROKER_IP` (the LAN IP of the PC running the
   broker). This file is gitignored; never commit it. The IP is baked into
   the firmware, so rebuild if the PC's IP changes.
2. Hotspot users (iPhone): turn on "Maximise Compatibility" and keep the
   hotspot screen open while the robot joins.

## Build

Needs the Arm GNU toolchain, the Pico SDK and GNU make 4.x.

    cd build_make
    gmake SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
          MQTT=1 DEBUG_MON=1 BENCH_NO_IMU=1 PICO_SDK_PATH=<path to pico-sdk> -j8

Output: `build_make/mtk3pico_smp0_usb_cdc_wifi_dhcp_mqtt.uf2`.

`BENCH_NO_IMU=1` lets the robot start without a working IMU (see "IMU" below).
Leave it out once the IMU works. Without it the robot will not start a
mission while the IMU fault is raised.

Other checks:
- Host unit tests: `make -C tests`
- Barr-C style check: `python3 tools/barr_check.py app_program`

## Flash

Hold BOOTSEL while plugging in USB, then copy the `.uf2` onto the `RPI-RP2` drive.

## Run / monitor (see docs/MQTT_SETUP.md)

    mosquitto -c tools/mosquitto_robot.conf -v
    python3 tools/tlm_decode.py monitor --broker <pc ip>
    python3 tools/tlm_decode.py tune speed_follow 160 --broker <pc ip>
    python3 tools/tlm_decode.py send map W left --version 1 --broker <pc ip>

Tunable live: speed_follow, steer_kp, steer_kd, steer_dir, enc_um_per_edge, ...
Live tunes reset on power cycle; copy good values into
`app_program/platform/robot_cfg.h`. The USB serial console also prints a
status line (line, motor, pitch, heading, sonar, barcode ADC levels, faults).

## What changed and what's verified

Verified on the robot (2026-11-02):
- MQTT both ways (telemetry, events, tune, map, ack).
- Encoders: these are motor-shaft encoders. Measured 1693 edges / 300 mm, so
  177 um per edge. Debounce 200 us. Wheel speed is now taken from the count
  rate over 50 ms.
- Line following: calibration sweep, then straight-line following at
  160 mm/s with no line loss. Steering Kp 65 / Kd 110 / dir +1. 600 ms slow
  start after every turn. Default follow speed is 130 mm/s.
- Barcode: switched to the IR module's **analogue** output on GP28 (ADC2)
  with software thresholding. The digital output cannot see narrow bars.
  One real read of sheet "W". Still marginal: the module must be mounted
  2-3 mm above the paper (a narrow bar must read > 800 static vs ~220 on white).
- The two sheets we were given decode as **Z** and **W**. The default map only
  knows A-D, so map them over MQTT.
- Obstacle: sonar distance and OBSTACLE events show up in telemetry.

Changes by area:
- comms (buddy1): trailing-newline and lose_link fixes; tlm_decode.py checks
  CONNACK on send and re-subscribes after a reconnect; new MQTT_SETUP.md and
  mosquitto_robot.conf; broker IP in wifi_credentials.
- motion (buddy2): brake timeout, stall detect, PID reset (motion.c);
  count-rate speed (encoder.c); encoder constants in robot_cfg.h; tune range.
- line/barcode (buddy3): analogue barcode pipeline (ir_hw.c ADC round robin,
  barcode.c edge finder: median-of-3 + 16-sample average, half-height edges,
  8 ms minimum bar, quiet 400 ms); Code 39 wide:narrow ratio 1.3; line_follow
  tweaks; Code 39 stray-edge unit test.
- obstacle (buddy5): obstacle.c fixes.
- integration/shared: vehicle.c (U-turn at line end, BENCH_NO_IMU, slow
  start), debug_mon.c (barcode levels / ADC stats), subdir.mk (BENCH_NO_IMU
  flag, lint stamp), GETTING_STARTED.md, barr_check.py.

## IMU (buddy4) - open issue

GY-511 (LSM303DLHC) on I2C0, SDA = GP0, SCL = GP1, addresses 0x19 (accel) and
0x1E (mag, identity 0x48). The driver is unchanged and retries once a second,
so the fault clears by itself as soon as the board answers.

Found today:
- The GY-511 has **no power LED**, so "LED is dark" does not mean the board is dead.
- A bus scan with the replacement board answered at 0x1E only with SDA and SCL
  **swapped** (SDA on GP1, SCL on GP0). So the signal wires were reversed. On a
  standard Grove cable yellow = SCL and white = SDA.
- After swapping the wires the robot still reported the IMU fault; not yet
  resolved. Next: check continuity of each wire to the GY-511 pins
  (VIN/GND/SCL/SDA), and that the cable is in the GP0/GP1 Grove socket.

## Other open items

- Barcode module needs its own mount at 2-3 mm (line sensors want to sit higher).
- Test junction turns (needs a barcode read + map).
- Curves at 160 mm/s; consider making 160 the default.
- Line following can lose the line right after calibration (starts at full
  speed a few degrees off); add a slow settle phase in vehicle.c.

## Split (file -> who commits it)

- buddy1: `app_program/buddy1_comms/*`, `tools/tlm_decode.py`,
  `docs/MQTT_SETUP.md`, `docs/MQTT_PROTOCOL.md`, `tools/mosquitto_robot.conf`,
  `config/wifi_credentials.example.h`, README broker-IP note.
- buddy2: `buddy2_motion/motion.c`, `buddy2_motion/encoder.c`, robot_cfg.h
  encoder lines, `platform/tune.c`.
- buddy3: `buddy3_line/*` (barcode, code39, ir_hw, line_follow), robot_cfg.h
  barcode/IR/steer/speed lines, `tests/test_main.c`, README barcode wiring row.
- buddy5: `buddy5_obstacle/obstacle.c`.
- integration/shared (to agree): `integration/vehicle.c`,
  `integration/debug_mon.c`, `build_make/mtkernel_3/app_program/subdir.mk`,
  `docs/GETTING_STARTED.md`, `tools/barr_check.py`.

`robot_cfg.h` is touched by buddies 1, 2 and 3: stage it in parts with `git add -p`.

## Lab-feedback fixes (2026-11-03, after the handoff above)

Committed on branch `claude/epic-hopper-p48qrj` of this repo, under `source/`.
`working_changes.patch` was NOT regenerated and does not include these.

| Feedback | File(s) | Change |
|---|---|---|
| Barcode "scans but not confirmed" | `buddy3_line/code39.c` | Bars and spaces are now classed separately (2 widest bars + widest space = wide). IR blur makes bars read wider and spaces narrower, so the old "3 widest of 9" mistook a narrow bar for the wide space. New unit test reproduces this (fails on the old code). |
| Decode front and back | `code39.c`, `tests/test_main.c` | Reverse read already existed; the new blur test checks every letter both ways. |
| Highlight decoded letter | `integration/debug_mon.c`, `tools/tlm_decode.py` | Banner with the letter, direction and junction action on USB console and MQTT monitor; "last X" on the status line; rejects print "seen but NOT decoded" with the reason. |
| Calibration sweep goes too far left / misses line from dead centre | `integration/vehicle.c` | Sweep is now 20 deg out, 40 deg back, then a return swing that STOPS when the centre sensor reaches the line edge (instead of a blind +25/-50/+25). Vehicle polls every 10 ms during that swing. |
| Obstacle: steers too far to the side; find width then decide turn | `buddy5_obstacle/obs_profile.c/.h`, `integration/vehicle.c`, `platform/robot_cfg.h` | Sonar beam spread taken off the width (`SONAR_BEAM_HALF_DEG`); sideways move measured from the real obstacle edge + half vehicle + `BYPASS_MARGIN_MM` (40), clamped 80..250 mm; detour angle = atan(sideways / distance ahead), 25..60 deg (was fixed 45). |
| Scans too fast | `buddy5_obstacle/obstacle.c`, `robot_cfg.h` | Fine-scan settle 60 -> 100 ms; first fine point gets the full 150 ms after the big servo jump. |

To tune on the robot: `SONAR_BEAM_HALF_DEG` (scan a box of known width),
`BYPASS_MARGIN_MM`, `CALIB_SWEEP_DEG` / `CALIB_RETURN_MAX_DEG` in vehicle.c.
Not touched: IMU wiring fault, junction turns, curve speed.
