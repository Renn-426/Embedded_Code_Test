# Autonomous Robot Car (INF2004 Embedded Systems)

Line-following robot car on a Raspberry Pi Pico W + Cytron Robo Pico, running
**micro T-Kernel 3.0** (RP2040 port by sirfonzie) and written in C to the
**Barr-C:2018** coding standard.

The robot follows a black line, reads Code 39 barcodes to decide turns
(A = left, B = right, C = straight, D = U-turn), measures hump height,
detects and drives around obstacles, and reports telemetry over MQTT.

## Team

| Buddy | Member | Responsibility |
|---|---|---|
| 1 | 2503687 | Communication: Wi-Fi, MQTT telemetry and commands |
| 2 | 2502437 | Motion: motors, wheel encoders, PID speed control |
| 3 | 2502515 | Line following and barcode decoding (IR sensors) |
| 4 | 2501723 | IMU (LSM303): tilt, heading, hump height |
| 5 | 2502046 | Obstacles: ultrasonic + servo scan, bypass |

Shared code: `app_program/platform/` (kernel objects, timing, bit packing,
settings) and `app_program/integration/` (vehicle state machine, barcode map).

## Wiring

| Part | Pico pin |
|---|---|
| Left motor / right motor | GP8, GP9 / GP10, GP11 (Robo Pico M1 / M2) |
| Left / right wheel encoder | GP26 / GP6 |
| Centre IR (AO) / left IR (DO) / barcode IR (AO) | GP27 / GP3 / GP28 |
| IMU LSM303 (I2C SDA / SCL) | GP0 / GP1 |
| Ultrasonic ECHO / TRIG | GP4 / GP5 |
| Servo | GP15 |
| Start button | GP20 (on board) |

All sensors are powered from 3.3 V. Pin settings are in
`app_program/platform/robot_cfg.h`.

## Build

Needs the Arm GCC toolchain, GNU make and the Pico SDK (we build in WSL/Ubuntu).

```
cd build_make
make SMP=0 CONSOLE=usb_cdc DEBUG_MON=1 PICO_SDK_PATH=~/pico/pico-sdk -j8
```

Full robot with Wi-Fi/MQTT (put your network in `config/wifi_credentials.h`
first, never commit it):

```
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 MQTT=1 PICO_SDK_PATH=~/pico/pico-sdk -j8
```

Single-buddy bench tests: add `BUDDY_TEST=2`, `3` or `23`.
Flash: hold BOOTSEL, plug in, copy the `.uf2` from `build_make` onto RPI-RP2.

## Monitoring

* USB: Serial Monitor shows the robot's state, sensors and faults (`DEBUG_MON=1`).
* MQTT: `python tools/tlm_decode.py monitor --broker <ip>`, or open
  `tools/control_panel.html` for live status, start/stop and live tuning.

## Credits

* micro T-Kernel RP2040 port: [sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040)
  (imported unchanged, then adapted to our wiring;).


---

# About the micro T-Kernel RP2040 port