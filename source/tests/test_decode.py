"""Cross-check: tools/tlm_decode.py against frames built by the C codec,
and C command parsing against frames built by the Python tool."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import tlm_decode  # noqa: E402

out = subprocess.check_output([os.path.join(HERE, 'codec_vectors')], text=True)
frames = dict(line.split() for line in out.strip().splitlines())
tlm = tlm_decode.decode(bytes.fromhex(frames['telemetry']))
evt = tlm_decode.decode(bytes.fromhex(frames['event']))
checks = [
    tlm['frame'] == 'TELEMETRY', tlm['seq'] == 42,
    tlm['vehicle']['state'] == 'LINE_FOLLOWING',
    tlm['vehicle']['map_version'] == 3,
    tlm['motion']['left_mm_s'] == -150,
    tlm['odometry']['distance_cm'] == 1234,
    tlm['line']['offset'] == 320,
    tlm['barcode']['letter'] == 'D',
    tlm['imu']['pitch_half_deg'] == 12, tlm['imu']['heading_deg'] == 270,
    tlm['hump']['max_mm'] == 17,
    tlm['obstacle']['last_plan'] == 'PASS_LEFT',
    tlm['comms']['link'] == 'ONLINE',
    tlm['health']['faults'] == ['IMU'],
    evt['event'] == 'MAP_ACK', evt['letter'] == 'B',
    evt['action'] == 'UTURN', evt['version'] == 2, evt['applied'] == 1,
    tlm_decode.build_command('start') == bytes([0x18, 0x00]),
    tlm_decode.build_tune('enc_um_per_edge', 5105) == bytes([0x18, 0x75,
                                                             0xF1, 0x13]),
    tlm_decode.build_tune('steer_dir', -1) == bytes([0x18, 0x65, 0xFF, 0xFF]),
    tlm_decode.build_command('map', 'C', 'right', 3) == bytes([0x18, 0x34,
                                                               0x22]),
]
failed = [index for index, ok in enumerate(checks) if not ok]
print('decoder cross-check: %d/%d passed' % (len(checks) - len(failed),
                                             len(checks)))
if failed:
    print('failed checks:', failed, tlm, evt)
sys.exit(1 if failed else 0)
