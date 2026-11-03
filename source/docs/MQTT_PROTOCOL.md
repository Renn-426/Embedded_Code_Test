# MQTT protocol (Buddy 1)

All payloads are compact binary (Change Requirement 1).  Fields are packed
least-significant bit first with no padding.  Every payload starts with a
header byte: bits 3..0 are the frame type and bits 7..4 the protocol version
(currently 1).  `tools/tlm_decode.py` decodes everything below and builds the
commands; `make -C tests` checks it against the firmware's own encoder.

The broker address (`MQTT_BROKER_IP`) is set in the local file
`config/wifi_credentials.h`; see [MQTT_SETUP.md](MQTT_SETUP.md).  Port and topic
root are in `app_program/platform/robot_cfg.h` (`MQTT_BROKER_PORT`,
`MQTT_TOPIC_ROOT`, default `inf2004/aai03/`).  QoS 0, no retain, keep-alive
10 s.

## Topics

| Topic | Direction | Frame | Rate |
|---|---|---|---|
| `inf2004/aai03/tlm` | robot to PC | TELEMETRY (type 0) | 5 Hz |
| `inf2004/aai03/hb` | robot to PC | HEARTBEAT (type 1); last will OFFLINE `0x15` | 1 Hz |
| `inf2004/aai03/evt` | robot to PC | EVENT (type 2) | on event |
| `inf2004/aai03/sum` | robot to PC | SUMMARY (type 3) | on request and at mission end |
| `inf2004/aai03/ack` | robot to PC | ACK (type 4) | per command received |
| `inf2004/aai03/cmd` | PC to robot | COMMAND (type 8) | as needed |

## Frames sent by the robot

| Frame | Fields after the header (bits) | Size |
|---|---|---|
| TELEMETRY | seq (8), then the 11 status words below | 36 B |
| HEARTBEAT | seq (8), uptime s (16), vehicle state (4), fault mask (8) | 6 B |
| EVENT | seq (8), event type (4), event data (28) | 6 B |
| SUMMARY | max hump mm (8), humps (6), barcodes read (8), rejects (8), obstacles profiled (6), distance cm (16), run time s (16), fault mask (8) | 11 B |
| ACK | command type (4), accepted (1) | 2 B |

The same 45 fields sent as JSON text would take several hundred bytes; the
binary frame is 36 bytes.

### Status words in a TELEMETRY frame (in this order)

| Word | Bits | Fields (LSB first) | Owner |
|---|---|---|---|
| VEHICLE | 23 | state 4, latched action 2, latched 1, map version 4, junctions 6, bypasses 4, retries 2 | Vehicle Control |
| MOTION | 26 | mode 3, left mm/s 11 (excess-1024), right mm/s 11 (excess-1024), stalled 1 | Buddy 2 |
| ENC_COUNTS | 32 | left edges 16, right edges 16 (wrapping) | Buddy 2 |
| ODOMETRY | 16 | distance cm 16 (wrapping) | Buddy 2 |
| LINE | 32 | offset 11 (excess-1024), centre ADC 12, left black 1, state 3, timestamp 5 | Buddy 3 |
| BARCODE | 27 | letter 5, reversed 1, seq 2, reads 8, rejects 8, state 3 | Buddy 3 |
| IMU | 27 | pitch 8 (0.5 degree units, excess-128), heading 9, motion class 3, hump state 2, valid 1, impacts 4 | Buddy 4 |
| HUMP | 22 | last hump mm 8, highest hump mm 8, count 6 | Buddy 4 |
| OBSTACLE | 24 | front distance cm 8 (255 = nothing), state 3, last plan 3, profiles 6, echo faults 4 | Buddy 5 |
| COMMS | 19 | link state 3, reconnections 8, dropped messages 8 | Buddy 1 |
| HEALTH | 24 | missed task periods 8, kernel errors 8, fault mask 8 | Buddy 1 |

The enumerations (states, plans, classes, link states) are listed in the
owners' headers and in `tools/tlm_decode.py`.

### Event types

| Type | Name | Data (LSB first) |
|---|---|---|
| 0 | BARCODE | letter 5 (0 = A), reversed 1, seq 2 |
| 1 | DECODE_FAIL | reason 3 (overflow, count, ratio, no match, frame, start), elements 6 |
| 2 | HUMP | height mm 8, highest mm 8 |
| 3 | IMPACT | peak acceleration in 16 mg units 8 |
| 4 | OBSTACLE | distance cm 8 |
| 5 | PROFILE | plan 3, width cm 8, distance cm 8 |
| 6 | STATE | new state 4, previous state 4 |
| 7 | FAULT | fault mask 8 |
| 8 | JUNCTION | action 2 |
| 9 | MAP_ACK | letter 5, action 2, map version 4, applied 1 |
| 10 | LINE | 1 = line lost, 0 = line found |
| 11 | TUNE | setting 4, new value 16 (excess-32768) |

Fault mask bits: 0 motor stall, 1 encoder, 2 IMU, 3 no echo, 4 WiFi loss,
5 IR calibration, 6 barcode FIFO overflow, 7 MQTT loss.

## Commands accepted by the robot

| Command | Bytes | Effect |
|---|---|---|
| `start` | `18 00` | READY to run (same as the START button) |
| `stop` | `18 01` | End the run: MISSION_COMPLETE and a summary |
| `reset` | `18 02` | SAFE_STOP or MISSION_COMPLETE back to READY |
| `summary` | `18 03` | Publish a SUMMARY frame |
| `map` | `18 v4 xx` | Set the action of one barcode letter (FR10) |
| `tune` | `18 s5 lo hi` | Change a setting live (s = setting id, value signed 16-bit little-endian). Lost on reset |

`map` layout: byte 1 is `0x04 | (version << 4)`; byte 2 is
`letter | (action << 5)` with letter 0 = A .. 25 = Z and action 0 = left,
1 = right, 2 = straight, 3 = U-turn.  Example: map C to right, version 3 is
`18 34 22`.

Every command is acknowledged on `ack` when it is received (accepted 1, or 0
if malformed).  A map update is acknowledged a second time, by the MAP_ACK
event, once Vehicle Control has applied it; the new map applies from the next
barcode decoded, never to a command already latched for the coming junction.

## Live tuning

`python3 tools/tlm_decode.py tune <setting> <value> --broker <ip>` changes a
setting while the robot runs.  The robot acknowledges on `ack` (accepted 0
if the value is out of range) and reports the new value as a TUNE event.
Changes are lost on reset: copy good values into `platform/robot_cfg.h`.

| Id | Setting | Range | Starts at (robot_cfg.h) |
|---|---|---|---|
| 0 | `motor_max` | 250..1000 permille | `MOTOR_MAX_PERMILLE` |
| 1 | `speed_follow` | 30..600 mm/s | `SPEED_FOLLOW_MM_S` |
| 2 | `speed_slow` | 30..600 mm/s (straight moves, reversing) | `SPEED_SLOW_MM_S` |
| 3 | `speed_turn` | 30..600 mm/s | `SPEED_TURN_MM_S` |
| 4 | `steer_kp` | 0..3000 milli | `STEER_KP_MILLI` |
| 5 | `steer_kd` | 0..3000 milli | `STEER_KD_MILLI` |
| 6 | `steer_dir` | -1 or 1 | `STEER_DIRECTION` |
| 7 | `enc_um_per_edge` | 500..30000 um | `ENC_UM_PER_EDGE` |
| 8 | `track_mm` | 50..300 mm (turn angle) | `TRACK_WIDTH_MM` |
| 9 | `reverse_mm` | 0..500 mm | `REVERSE_DISTANCE_MM` |

## Link recovery

WiFi is joined by the port's radio task at boot.  If the link is lost, the
network side re-joins at most every 15 s; the MQTT session is retried after 1,
2, then 4 s.  While offline only the newest message of each topic is kept.
Loss of the broker is detected within 15 s (1.5 x keep-alive), and telemetry
resumes within about 10 s once the broker is back (NFR16).  The robot keeps
running throughout.

## PC tool

```sh
pip install paho-mqtt
python3 tools/tlm_decode.py monitor --broker 192.168.1.10
python3 tools/tlm_decode.py send start --broker 192.168.1.10
python3 tools/tlm_decode.py send map C right --version 3 --broker 192.168.1.10
python3 tools/tlm_decode.py decode 10 2a ...
```
