#!/usr/bin/env python3
"""PC side of the AAI03 robot MQTT protocol (Buddy 1).

Decodes the compact binary frames the robot publishes and builds the binary
command frames it accepts.  The bit layouts mirror app_program/buddy1_comms/
tlm_codec.h and the owner headers named below; fields are packed
least-significant bit first with no padding (Change Requirement 1).

Usage:
  python3 tools/tlm_decode.py decode 10 07 ...        decode one payload (hex)
  python3 tools/tlm_decode.py monitor --broker 192.168.1.10
  python3 tools/tlm_decode.py send start --broker 192.168.1.10
  python3 tools/tlm_decode.py send map B uturn --version 2 --broker ...

monitor and send need the paho-mqtt package (pip install paho-mqtt).
"""

import argparse
import sys
import threading

TOPIC_ROOT = 'inf2004/aai03/'
VERSION = 1

VEHICLE_STATES = ['INITIALISING', 'READY', 'CALIBRATING', 'LINE_FOLLOWING',
                  'JUNCTION_ACTION', 'OBSTACLE_SCAN', 'BYPASS',
                  'LINE_REACQUIRE', 'REVERSE_RETRY', 'SAFE_STOP',
                  'MISSION_COMPLETE']
ACTIONS = ['LEFT', 'RIGHT', 'STRAIGHT', 'UTURN']
MOTION_MODES = ['IDLE', 'FOLLOWING', 'MOVING', 'TURNING', 'BRAKING',
                'STALLED']
LINE_STATES = ['CALIBRATING', 'TRACKING', 'JUNCTION', 'LOST', 'SEARCHING']
BARCODE_STATES = ['IDLE', 'CAPTURING', 'DECODING', 'REPORT', 'REJECT']
IMU_CLASSES = ['STATIONARY', 'CRUISING', 'ACCELERATING', 'DECELERATING',
               'TURNING', 'CLIMBING', 'DESCENDING', 'IMPACT']
HUMP_STATES = ['LEVEL', 'CLIMBING', 'DESCENDING']
OBSTACLE_STATES = ['MONITOR', 'DETECTED', 'COARSE_SCAN', 'FINE_SCAN',
                   'PROFILE', 'ECHO_FAULT']
PLANS = ['CONTINUE', 'STOP', 'PASS_LEFT', 'PASS_RIGHT', 'REVERSE']
LINKS = ['WIFI_INIT', 'WIFI_CONNECTING', 'MQTT_CONNECTING', 'ONLINE',
         'LOST', 'RADIO_FAULT', 'DISABLED']
FAULTS = ['MOTOR_STALL', 'ENCODER', 'IMU', 'NO_ECHO', 'WIFI_LOSS',
          'IR_CALIB', 'BARCODE_OVF', 'MQTT_LOSS']
REJECTS = ['OVERFLOW', 'COUNT', 'RATIO', 'NO_MATCH', 'FRAME', 'START']
EVENTS = ['BARCODE', 'DECODE_FAIL', 'HUMP', 'IMPACT', 'OBSTACLE', 'PROFILE',
          'STATE', 'FAULT', 'JUNCTION', 'MAP_ACK', 'LINE', 'TUNE']
COMMANDS = {'start': 0, 'stop': 1, 'reset': 2, 'summary': 3, 'map': 4}
TUNE_COMMAND = 5
# Live-tunable settings, in app_program/platform/tune.h order (= MQTT id).
TUNE_PARAMS = ['motor_max', 'speed_follow', 'speed_slow', 'speed_turn',
               'steer_kp', 'steer_kd', 'steer_dir', 'enc_um_per_edge',
               'track_mm', 'reverse_mm']

# (field name, width, kind) per status word, in sys_status_word_t order.
# kind: 'u' unsigned, 's' excess-K signed, or a list of names.
STATUS_WORDS = [
    ('vehicle', [('state', 4, VEHICLE_STATES), ('latched_action', 2, ACTIONS),
                 ('latched', 1, 'u'), ('map_version', 4, 'u'),
                 ('junctions', 6, 'u'), ('bypasses', 4, 'u'),
                 ('retries', 2, 'u')]),
    ('motion', [('mode', 3, MOTION_MODES), ('left_mm_s', 11, 's'),
                ('right_mm_s', 11, 's'), ('stalled', 1, 'u')]),
    ('enc_counts', [('left', 16, 'u'), ('right', 16, 'u')]),
    ('odometry', [('distance_cm', 16, 'u')]),
    ('line', [('offset', 11, 's'), ('centre_raw', 12, 'u'),
              ('left_black', 1, 'u'), ('state', 3, LINE_STATES),
              ('stamp', 5, 'u')]),
    ('barcode', [('letter', 5, 'letter'), ('reversed', 1, 'u'),
                 ('seq', 2, 'u'), ('reads', 8, 'u'), ('rejects', 8, 'u'),
                 ('state', 3, BARCODE_STATES)]),
    ('imu', [('pitch_half_deg', 8, 's'), ('heading_deg', 9, 'u'),
             ('class', 3, IMU_CLASSES), ('hump', 2, HUMP_STATES),
             ('valid', 1, 'u'), ('impacts', 4, 'u')]),
    ('hump', [('last_mm', 8, 'u'), ('max_mm', 8, 'u'), ('count', 6, 'u')]),
    ('obstacle', [('front_cm', 8, 'u'), ('state', 3, OBSTACLE_STATES),
                  ('last_plan', 3, PLANS), ('profiles', 6, 'u'),
                  ('echo_faults', 4, 'u')]),
    ('comms', [('link', 3, LINKS), ('reconnects', 8, 'u'),
               ('dropped', 8, 'u')]),
    ('health', [('missed_periods', 8, 'u'), ('kernel_errors', 8, 'u'),
                ('faults', 8, 'faults')]),
]

EVENT_LAYOUTS = {
    'BARCODE': [('letter', 5, 'letter'), ('reversed', 1, 'u'),
                ('seq', 2, 'u')],
    'DECODE_FAIL': [('reason', 3, REJECTS), ('elements', 6, 'u')],
    'HUMP': [('height_mm', 8, 'u'), ('max_mm', 8, 'u')],
    'IMPACT': [('peak_16mg', 8, 'u')],
    'OBSTACLE': [('distance_cm', 8, 'u')],
    'PROFILE': [('plan', 3, PLANS), ('width_cm', 8, 'u'),
                ('distance_cm', 8, 'u')],
    'STATE': [('new', 4, VEHICLE_STATES), ('previous', 4, VEHICLE_STATES)],
    'FAULT': [('faults', 8, 'faults')],
    'JUNCTION': [('action', 2, ACTIONS)],
    'MAP_ACK': [('letter', 5, 'letter'), ('action', 2, ACTIONS),
                ('version', 4, 'u'), ('applied', 1, 'u')],
    'LINE': [('lost', 1, 'u')],
    'TUNE': [('setting', 4, TUNE_PARAMS), ('value', 16, 's')],
}


class BitReader:
    """Reads fields least-significant bit first from a byte string."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def get(self, width):
        value = 0
        for bit in range(width):
            index = self.pos + bit
            if index // 8 >= len(self.data):
                raise ValueError('frame too short')
            if (self.data[index // 8] >> (index % 8)) & 1:
                value |= 1 << bit
        self.pos += width
        return value


def interpret(value, width, kind):
    """Turn a raw field into a readable value."""
    if kind == 's':
        return value - (1 << (width - 1))
    if kind == 'letter':
        return chr(ord('A') + value) if value < 26 else '?'
    if kind == 'faults':
        return [name for bit, name in enumerate(FAULTS) if value & (1 << bit)]
    if isinstance(kind, list):
        return kind[value] if value < len(kind) else value
    return value


def read_fields(reader, layout):
    """Read a list of (name, width, kind) fields."""
    return {name: interpret(reader.get(width), width, kind)
            for name, width, kind in layout}


def decode(payload):
    """Decode one robot payload into a dictionary."""
    reader = BitReader(payload)
    frame_type = reader.get(4)
    version = reader.get(4)
    if frame_type == 5:
        return {'frame': 'OFFLINE (last will)'}
    if version != VERSION:
        return {'frame': 'unknown version %d' % version}
    if frame_type == 0:
        result = {'frame': 'TELEMETRY', 'seq': reader.get(8)}
        for name, layout in STATUS_WORDS:
            result[name] = read_fields(reader, layout)
        return result
    if frame_type == 1:
        return {'frame': 'HEARTBEAT', 'seq': reader.get(8),
                'uptime_s': reader.get(16),
                'state': interpret(reader.get(4), 4, VEHICLE_STATES),
                'faults': interpret(reader.get(8), 8, 'faults')}
    if frame_type == 2:
        seq = reader.get(8)
        kind = interpret(reader.get(4), 4, EVENTS)
        data = BitReader(reader.get(28).to_bytes(4, 'little'))
        fields = read_fields(data, EVENT_LAYOUTS.get(kind, []))
        return {'frame': 'EVENT', 'seq': seq, 'event': kind, **fields}
    if frame_type == 3:
        return {'frame': 'SUMMARY', 'max_hump_mm': reader.get(8),
                'humps': reader.get(6), 'barcodes': reader.get(8),
                'rejects': reader.get(8), 'obstacles': reader.get(6),
                'distance_cm': reader.get(16), 'run_time_s': reader.get(16),
                'faults': interpret(reader.get(8), 8, 'faults')}
    if frame_type == 4:
        command = reader.get(4)
        names = {value: key for key, value in COMMANDS.items()}
        names[TUNE_COMMAND] = 'tune'
        return {'frame': 'ACK', 'command': names.get(command, command),
                'accepted': bool(reader.get(1))}
    return {'frame': 'unknown type %d' % frame_type}


def highlight(decoded):
    """Print a banner for a decoded barcode, so it stands out in the log."""
    if decoded.get('event') == 'BARCODE':
        letter = decoded.get('letter', '?')
        way = 'BACKWARDS' if decoded.get('reversed') else 'FORWARDS'
        bold = '\033[1;30;43m' if sys.stdout.isatty() else ''
        reset = '\033[0m' if sys.stdout.isatty() else ''
        lines = ['=' * 34,
                 '== %-28s ==' % ('BARCODE DECODED:  [ %s ]' % letter),
                 '== %-28s ==' % ('read ' + way),
                 '=' * 34]
        for line in lines:
            print('%s%s%s' % (bold, line, reset))
        sys.stdout.flush()
    elif decoded.get('event') == 'DECODE_FAIL':
        print('  -- barcode seen but NOT decoded: %s, %s elements'
              % (decoded.get('reason'), decoded.get('elements')), flush=True)


def build_command(name, letter=None, action=None, map_version=0):
    """Build a binary command payload."""
    header = 0x08 | (VERSION << 4)
    body = COMMANDS[name] | ((map_version & 0x0F) << 4)
    payload = bytes([header, body])
    if name == 'map':
        index = ord(letter.upper()) - ord('A')
        if not 0 <= index < 26:
            raise ValueError('letter must be A..Z')
        payload += bytes([index | (ACTIONS.index(action.upper()) << 5)])
    return payload


def build_tune(setting, value):
    """Build a TUNE command payload: setting name and signed 16-bit value."""
    index = TUNE_PARAMS.index(setting)
    if not -32768 <= value <= 32767:
        raise ValueError('value must fit in 16 bits')
    header = 0x08 | (VERSION << 4)
    return bytes([header, TUNE_COMMAND | (index << 4)]) + \
        (value & 0xFFFF).to_bytes(2, 'little')


def run_mqtt(args):
    """monitor or send over MQTT (needs paho-mqtt)."""
    import paho.mqtt.client as mqtt

    # paho-mqtt 2.x requires the callback API version; 1.x has no such
    # argument.  Version 1 callbacks match on_message below.
    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    else:
        client = mqtt.Client()
    if args.mode in ('send', 'tune'):
        connected = threading.Event()
        connack = {}

        def on_connect(_client, _userdata, _flags, rc):
            connack['rc'] = rc
            connected.set()

        client.on_connect = on_connect
        client.connect(args.broker, args.port, 10)
        client.loop_start()
        if not connected.wait(5.0):
            client.loop_stop()
            print('no reply from broker %s' % args.broker, file=sys.stderr)
            return 1
        if connack['rc'] != 0:
            client.loop_stop()
            print('broker %s refused the connection: %s'
                  % (args.broker, mqtt.connack_string(connack['rc'])),
                  file=sys.stderr)
            return 1
        if args.mode == 'tune':
            payload = build_tune(args.setting, args.value)
            args.command = '%s=%d' % (args.setting, args.value)
        else:
            payload = build_command(args.command, args.letter, args.action,
                                    args.version)
        client.publish(TOPIC_ROOT + 'cmd', payload, qos=0).wait_for_publish(5.0)
        client.disconnect()
        client.loop_stop()
        print('sent %s: %s' % (args.command, payload.hex(' ')))
        return 0

    def on_message(_client, _userdata, message):
        try:
            decoded = decode(message.payload)
            print(message.topic, decoded, flush=True)
            highlight(decoded)
        except ValueError as error:
            print(message.topic, 'undecodable:', error,
                  message.payload.hex(' '))

    def on_connect(mqtt_client, _userdata, _flags, rc):
        # Subscribe here, not after connect(): paho reconnects on its own
        # after a dropped link but does not restore subscriptions made
        # outside this callback, so the monitor would go silent for good.
        print('connected to %s: %s' % (args.broker, mqtt.connack_string(rc)),
              flush=True)
        if rc == 0:
            mqtt_client.subscribe(TOPIC_ROOT + '#')

    def on_disconnect(_client, _userdata, rc):
        print('disconnected from broker (rc=%s), reconnecting' % rc,
              file=sys.stderr, flush=True)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect(args.broker, args.port, 10)
    client.loop_forever(retry_first_connection=True)
    return 0


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='mode', required=True)
    dec = sub.add_parser('decode')
    dec.add_argument('hex', nargs='+')
    mon = sub.add_parser('monitor')
    snd = sub.add_parser('send')
    snd.add_argument('command', choices=sorted(COMMANDS))
    snd.add_argument('letter', nargs='?')
    snd.add_argument('action', nargs='?', choices=[a.lower() for a in ACTIONS]
                     + ACTIONS)
    snd.add_argument('--version', type=int, default=0)
    tun = sub.add_parser('tune', help='change a setting live; lost on reset')
    tun.add_argument('setting', choices=TUNE_PARAMS)
    tun.add_argument('value', type=int)
    for sub_parser in (mon, snd, tun):
        sub_parser.add_argument('--broker', required=True)
        sub_parser.add_argument('--port', type=int, default=1883)
    args = parser.parse_args(argv[1:])
    if args.mode == 'decode':
        print(decode(bytes.fromhex(''.join(args.hex))))
        return 0
    return run_mqtt(args)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
