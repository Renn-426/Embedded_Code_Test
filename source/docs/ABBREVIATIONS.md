# Project abbreviations (Barr-C rule 1.5.b)

Abbreviations from Barr-C Appendix A (adc, avg, b_, buf, cfg, curr, err, g_,
gpio, h_, init, io, isr, led, max, min, msec, msg, num, p_, pp_, prev, prio,
pwm, q, reg, rx, sem, str, sync, tmp, tx, usec) are used with their standard
meaning.  The project adds:

| Abbreviation | Meaning |
|---|---|
| ack | acknowledgement |
| ao / do | analogue output / digital output of an IR module |
| cm, mm, um | centimetre, millimetre, micrometre (units in names) |
| cmd | command |
| ctx | context (a module's working state) |
| deg, mdeg | degree, millidegree |
| enc | wheel encoder |
| evt | event |
| ff | feed-forward |
| flg | micro T-Kernel event flag |
| hb | heartbeat |
| hw | hardware (module suffix `_hw`: uses the Pico SDK) |
| imu | inertial measurement unit (LSM303DLHC) |
| ipc | inter-task communication |
| ir | infrared |
| lwt | MQTT last will and testament |
| mbf | micro T-Kernel message buffer |
| mg | milli-g (acceleration) |
| ms, us | millisecond, microsecond (units in names) |
| mtx | micro T-Kernel mutex |
| nav | navigation |
| net | network (module suffix `_net`: uses lwIP and the radio) |
| obs | obstacle |
| q15 | fixed-point number with 15 fractional bits (1.0 = 32768) |
| sq | square |
| tlm | telemetry |
| ovf | overflow |
