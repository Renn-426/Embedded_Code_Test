# MQTT setup

The robot joins Wi-Fi, connects to a Mosquitto broker on a PC, and publishes
telemetry. The PC watches it and sends commands with `tools/tlm_decode.py`.
The frames are described in [MQTT_PROTOCOL.md](MQTT_PROTOCOL.md).

The robot and the PC must be on the same network. The Pico W only supports
2.4 GHz Wi-Fi.

## 1. Install on the PC

macOS:

```sh
brew install mosquitto
python3 -m pip install --user paho-mqtt
```

Ubuntu or WSL:

```sh
sudo apt install mosquitto mosquitto-clients
python3 -m pip install --user paho-mqtt
```

Under WSL the robot cannot reach the broker by default. Run the broker on
Windows, or forward port 1883 to WSL.

## 2. Start the broker

Find the PC's address first (macOS: `ipconfig getifaddr en0`; Windows:
`ipconfig`; Linux: `hostname -I`). Then, from the repository root:

```sh
mosquitto -c tools/mosquitto_robot.conf -v
```

On macOS use `/opt/homebrew/opt/mosquitto/sbin/mosquitto` if `mosquitto` is not
found. Allow incoming connections if the firewall asks.

## 3. Set the network details

```sh
cp config/wifi_credentials.example.h config/wifi_credentials.h
```

In `config/wifi_credentials.h`, set `UTK_WIFI_CREDENTIALS_SET` to 1 and fill in
`UTK_WIFI_SSID`, `UTK_WIFI_PASSWORD` and `MQTT_BROKER_IP` (the PC's address).
Git ignores this file, so never commit a copy of it. The address can change when
the PC rejoins a network: rebuild if it does.

## 4. Build and flash

From `build_make`:

```sh
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 MQTT=1 DEBUG_MON=1 PICO_SDK_PATH=~/pico/pico-sdk -j8
```

On macOS, run `brew install make` and use `gmake`: the built-in GNU Make 3.81 picks the wrong rule for the USB console files.

Hold BOOTSEL, plug the Pico in, and copy
`mtk3pico_smp0_usb_cdc_wifi_dhcp_mqtt.uf2` onto the drive.

## 5. Check it works

In a second terminal, replacing the address:

```sh
python3 tools/tlm_decode.py monitor --broker 192.168.1.10
```

Telemetry should appear within about 10 s of power-on. Mosquitto's log shows
`aai03-robot` connecting. Send commands from a third terminal:

```sh
python3 tools/tlm_decode.py send start --broker 192.168.1.10
python3 tools/tlm_decode.py send stop --broker 192.168.1.10
python3 tools/tlm_decode.py send summary --broker 192.168.1.10
python3 tools/tlm_decode.py send map A uturn --version 1 --broker 192.168.1.10
python3 tools/tlm_decode.py tune track_mm 130 --broker 192.168.1.10
```

## Live calibration

`tune` changes a setting without rebuilding, so the robot can be calibrated over
MQTT. Changes are lost on reset, so copy good values into
`app_program/platform/robot_cfg.h`. Useful settings are `enc_um_per_edge`,
`track_mm` and `steer_dir`.

## If it does not connect

| Symptom | Check |
|---|---|
| Nothing in the Mosquitto log | Same network, 2.4 GHz, correct `MQTT_BROKER_IP`, PC firewall |
| `monitor` cannot connect | Broker running, correct address after `--broker` |
| Fault line `WiFi lost` on USB | SSID and password, signal, WPA2 network |
| Fault line `MQTT lost` on USB | Broker stopped or address changed |
