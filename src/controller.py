#!/usr/bin/env python3
"""BL-USBMach HID controller: identification, experimental jog and stop."""
import argparse
import json
import math
from pathlib import Path
import time
import struct
import sys
from contextlib import nullcontext

VID, PID = 0x3698, 0x0107
REPORT_SIZE = 64
MAX_MOVE_COUNTS = 10000
MAX_RATE = 1000
MAX_ACCELERATION = 10000
LAST_JOG_FILE = Path(__file__).resolve().parents[1] / '.last-jog-report.hex'
# Initial wire bytes reproduced from DLL function 0x10006c80 with zeroed buffer.
IDENTIFY_REPORT = bytes.fromhex(
    "0000000000000000000000000000000000000000000000000000000000000000000000000000000000b6000000000000000000bd0000000000000000001fba62"
)


def jog_report(axis, steps, rate, acceleration):
    """Experimental jog command from DLL 0x10006ba0, plain firmware."""
    if axis not in range(5):
        raise ValueError('axis must be 0..4 (X, Y, Z, A, B)')
    if not 1 <= abs(steps) <= MAX_MOVE_COUNTS:
        raise ValueError('steps must be nonzero, at most 10000 in magnitude')
    if not 1 <= rate <= MAX_RATE or not 1 <= acceleration <= MAX_ACCELERATION:
        raise ValueError('rate must be 1..1000; acceleration must be 1..10000')
    payload = bytearray(REPORT_SIZE)
    struct.pack_into('<7I', payload, 0, axis, int(steps >= 0), abs(steps),
                     rate, acceleration, 0, 0)
    payload[63] = 0x0b
    return bytes(payload)


def stop_report(previous=None):
    """DLL JogOff changes only the command byte of the current buffer."""
    if previous is None:
        previous = bytes(REPORT_SIZE)
    if len(previous) != REPORT_SIZE:
        raise ValueError('Previous report must contain exactly 64 bytes')
    payload = bytearray(previous)
    payload[63] = 0x0c
    return bytes(payload)


def metric_jog_report(axis, distance_mm, steps_per_mm, feed_mm_min, acceleration_mm_s2):
    """Convert metric inputs to DLL jog fields; completion requires polling."""
    values = (distance_mm, steps_per_mm, feed_mm_min, acceleration_mm_s2)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Metric parameters must be finite')
    if distance_mm == 0 or min(steps_per_mm, feed_mm_min, acceleration_mm_s2) <= 0:
        raise ValueError('Distance must be nonzero; calibration, feed and acceleration positive')
    converted = (distance_mm * steps_per_mm, feed_mm_min * steps_per_mm / 60,
                 acceleration_mm_s2 * steps_per_mm)
    if not all(math.isfinite(v) for v in converted):
        raise ValueError('Converted metric parameters are too large')
    steps, rate, acceleration = map(math.trunc, converted)
    payload = jog_report(axis, steps, rate, acceleration)
    return payload, {'steps': steps, 'rate': rate, 'acceleration': acceleration,
                     'quantized_distance_mm': steps / steps_per_mm,
                     'quantized_feed_mm_min': rate * 60 / steps_per_mm,
                     'quantized_acceleration_mm_s2': acceleration / steps_per_mm}


def status_report(previous):
    """DLL 0x10003a00 polls using its separate output/status buffer."""
    if len(previous) != REPORT_SIZE:
        raise ValueError('Status template must contain exactly 64 bytes')
    payload = bytearray(previous)
    payload[63] = 3
    return bytes(payload)


def poll_report(output_mask, z_steps_per_unit, spindle_percent=0, spindle_on=False,
                estop=False, mpg_mode=False, jog_increment=0, soft_limits=False):
    """Construct DLL output/status buffer; outputs follow configured board mapping."""
    if not isinstance(output_mask, int) or not 0 <= output_mask < (1 << 30):
        raise ValueError('output mask must contain only bits 0..29')
    if not isinstance(z_steps_per_unit, int) or not 0 <= z_steps_per_unit <= 65535:
        raise ValueError('Z steps per unit must be an integer in 0..65535')
    if not isinstance(spindle_percent, int) or not 0 <= spindle_percent <= 100:
        raise ValueError('spindle percent must be an integer in 0..100')
    if not math.isfinite(jog_increment) or not 0 <= jog_increment <= 3.4e38:
        raise ValueError('jog increment must be finite, nonnegative and fit float32')
    payload = bytearray(REPORT_SIZE)
    struct.pack_into('<IBBH', payload, 0, output_mask, spindle_percent,
                     int(spindle_on and not estop), z_steps_per_unit)
    payload[8] = int(estop)
    for flag_offset, increment_offset in ((14, 15), (19, 20)):
        payload[flag_offset] = int(mpg_mode)
        struct.pack_into('<f', payload, increment_offset, jog_increment)
    payload[24] = payload[26] = int(soft_limits)
    payload[63] = 3
    return bytes(payload)


def decode_status(report):
    """Candidate fields from DLL poll parser; no verified response discriminator."""
    if len(report) != REPORT_SIZE:
        raise ValueError('Status response must contain exactly 64 bytes')
    return {'raw': report.hex(), 'candidate': True,
            'axis_counters': dict(zip('XYZAB', struct.unpack_from('<5i', report, 4))),
            'dll_ready_bit': bool(report[1] & 0x80),
            'flags_offset_40': report[40], 'command_byte': report[63]}


def tuning_report(axes, flags):
    if len(axes) != 5 or len(flags) != 2 or any(type(v) is not int or not 0 <= v <= 255 for v in flags):
        raise ValueError('Tuning requires five axes and two explicit flag bytes')
    payload = bytearray(64)
    for index, axis in enumerate(axes):
        scale, feed, accel = (axis[k] for k in ('steps_per_unit', 'feed_units_min', 'acceleration_units_s2'))
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in (scale, feed, accel)):
            raise ValueError('Axis tuning values must be finite and positive')
        values = (math.trunc(scale), math.trunc(scale * feed / 60), math.trunc(scale * accel))
        if not 1 <= values[0] <= 65535 or not all(1 <= v <= 0xffffffff for v in values[1:]):
            raise ValueError('Tuning values exceed wire field bounds')
        struct.pack_into('<H', payload, index * 2, values[0])
        struct.pack_into('<I', payload, 10 + index * 4, values[1])
        struct.pack_into('<I', payload, 30 + index * 4, values[2])
    payload[50:52] = bytes(flags)
    payload[63] = 0x0e
    return bytes(payload)


def load_profile(path):
    profile = json.loads(path.read_text())
    def packet(key, command):
        value = bytes.fromhex(profile[key])
        if len(value) != 64 or value[63] != command:
            raise ValueError(f'{key} must be a 64-byte command 0x{command:02x}')
        return value
    # Unknown configuration fields stay explicit; never copy captured stack garbage.
    setup = [packet('identification_report', 0x62), packet('ack_report', 2),
             tuning_report(profile['axes'], profile['tuning_flags']),
             packet('pin_map_report', 0x0f)]
    polling = poll_report(profile['outputs_mask'], math.trunc(profile['axes'][2]['steps_per_unit']))
    return setup, polling


def capture_report(capture, report, phase=None):
    if capture is not None and report:
        entry = {'time_ns': time.time_ns(), 'hex': report.hex()}
        if phase is not None:
            entry['phase'] = phase
        capture.write(json.dumps(entry) + '\n')
        capture.flush()


def exchange_status(board, payload, timeout_ms=500, capture=None):
    board.send_verified_report(payload)
    deadline = time.monotonic() + timeout_ms / 1000
    while (remaining := deadline - time.monotonic()) > 0:
        report = board.read(max(1, min(50, math.ceil(remaining * 1000))))
        capture_report(capture, report)
        if len(report) == 64 and report[63] == 3:
            return decode_status(report)
    raise RuntimeError('No matching 0x03 status response before timeout')


def polled_jog(board, payload, seconds, polling, capture=None):
    # Verify the poll exchange before starting. One request in flight at a time.
    initial = exchange_status(board, polling, capture=capture)
    axis, direction, steps = struct.unpack_from('<3I', payload)
    axis_name = 'XYZAB'[axis]
    target = (initial['axis_counters'][axis_name] + (steps if direction else -steps)) & 0xffffffff
    try:
        board.send_verified_report(payload)
        deadline = time.monotonic() + seconds
        while (remaining := deadline - time.monotonic()) > 0:
            status = exchange_status(board, polling, min(200, max(1, math.ceil(remaining * 1000))), capture)
            if status['dll_ready_bit'] and (status['axis_counters'][axis_name] & 0xffffffff) == target:
                return True
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(.1, remaining))
        return False
    finally:
        board.stop(payload)


def timed_jog(board, payload, seconds, capture=None):
    """Attempt JogOff after timeout, interruption, or a failed jog write."""
    try:
        board.send_verified_report(payload)
        if capture is None:
            time.sleep(seconds)
        else:
            deadline = time.monotonic() + seconds
            while (remaining := deadline - time.monotonic()) > 0:
                report = board.read(max(1, min(50, math.ceil(remaining * 1000))))
                capture_report(capture, report)
    finally:
        board.stop(payload)


def initialize_board(board, setup, polling):
    identity = board.identify(setup[0])
    if identity['firmware_code'] != 21:
        raise RuntimeError('Initialization supports only plain firmware 21')
    for report in setup[1:]:
        board.send_verified_report(report)
        time.sleep(.03)
    return {'identity': identity, 'status': exchange_status(board, polling)}


def console(board, profile, polling):
    """Blocking manual commands; work zero lasts only for this session."""
    offsets = dict.fromkeys('XYZ', 0)
    previous = None
    help_text = 'status | zero [X/Y/Z] | jog X/Y/Z signed_mm [timeout_seconds] | stop | quit'
    print(help_text)
    print('During a move, Ctrl-C attempts JogOff and returns to the prompt.')
    try:
        while True:
            try:
                words = input('cnc> ').split()
                if not words:
                    continue
                command = words[0].lower()
                if command in ('quit', 'exit') and len(words) == 1:
                    break
                if command == 'help':
                    print(help_text)
                elif command == 'stop' and len(words) == 1:
                    board.stop(previous)
                    print('JogOff transmitted.')
                elif command in ('status', 'zero'):
                    if len(words) > (2 if command == 'zero' else 1):
                        raise ValueError(help_text)
                    axes = words[1].upper() if len(words) == 2 else 'XYZ'
                    if axes not in ('X', 'Y', 'Z', 'XYZ'):
                        raise ValueError('Choose X, Y or Z')
                    counters = exchange_status(board, polling)['axis_counters']
                    if command == 'zero':
                        for axis in axes:
                            offsets[axis] = counters[axis]
                    positions = {}
                    for i, axis in enumerate('XYZ'):
                        delta = (counters[axis] - offsets[axis] + 2**31) % 2**32 - 2**31
                        positions[axis] = delta / profile['axes'][i]['steps_per_unit']
                    print('Reported work position (mm): ' + ' '.join(f'{axis}={value:.3f}' for axis, value in positions.items()))
                elif command == 'jog' and len(words) in (3, 4):
                    axis = words[1].upper()
                    if axis not in ('X', 'Y', 'Z'):
                        raise ValueError('Choose X, Y or Z')
                    seconds = float(words[3]) if len(words) == 4 else 15
                    if not math.isfinite(seconds) or not 0 < seconds <= 30:
                        raise ValueError('Timeout must be greater than zero and at most 30 seconds')
                    index = 'XYZ'.index(axis)
                    settings = profile['axes'][index]
                    payload, _ = metric_jog_report(index, float(words[2]), settings['steps_per_unit'],
                                                   settings['feed_units_min'], settings['acceleration_units_s2'])
                    LAST_JOG_FILE.write_text(payload.hex() + '\n')
                    previous = payload
                    completed = polled_jog(board, payload, seconds, polling)
                    print('Counter target reached; board ready.' if completed else 'Timed out; JogOff transmitted.')
                else:
                    raise ValueError(help_text)
            except ValueError as exc:
                print(exc)
            except KeyboardInterrupt:
                board.stop(previous)
                print('\nJogOff transmitted.')
            except EOFError:
                break
    finally:
        board.stop(previous)


def devices():
    # sysfs enumeration works even when this sandbox hides /dev/hidraw.
    root = Path('/sys/class/hidraw')
    if root.exists():
        result = []
        for entry in root.glob('hidraw*'):
            info = entry.joinpath('device/uevent').read_text()
            if '00003698:00000107' in info.upper():
                result.append({'path': '/dev/' + entry.name, 'details': info.strip(),
                               'report_descriptor': entry.joinpath('device/report_descriptor').read_bytes().hex()})
        return result
    import hid
    return [{**d, 'path': d['path'].decode(errors='replace')} for d in hid.enumerate(VID, PID)]


class Controller:
    def __init__(self, path=None):
        try:
            import hid
        except ImportError as exc:
            raise RuntimeError('Install HID transport: python -m pip install hidapi') from exc
        self.device = hid.device()
        try:
            if path:
                self.device.open_path(path.encode())
            else:
                self.device.open(VID, PID)
        except OSError as exc:
            self.device.close()
            raise RuntimeError(
                'Cannot open board: check USB connection and HID permissions. '
                + ('On Linux, device permissions may require sudo or a udev rule.'
                   if sys.platform.startswith('linux') else
                   'On macOS, close other applications using the board and retry.')
            ) from exc

    def read(self, timeout_ms=100):
        return bytes(self.device.read(REPORT_SIZE, timeout_ms))

    def send_verified_report(self, payload):
        if len(payload) != REPORT_SIZE:
            raise ValueError('Expected exactly 64 payload bytes, excluding the HID report ID')
        # Descriptor has no report IDs; hidapi requires a leading zero for writes.
        if self.device.write(b'\x00' + payload) != REPORT_SIZE + 1:
            raise RuntimeError('Incomplete HID write')

    def identify(self, payload=IDENTIFY_REPORT):
        self.send_verified_report(payload)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            report = self.read()
            if len(report) == REPORT_SIZE and report[63] == 0x62:
                return {'raw': report.hex(),
                        'firmware_code': struct.unpack_from('<I', report, 20)[0],
                        'device_number': struct.unpack_from('<I', report, 4)[0]}
        raise RuntimeError('No matching identification response within two seconds')

    def close(self):
        self.device.close()

    def stop(self, previous=None):
        """Transmit candidate JogOff; acknowledgement is not yet decoded."""
        self.send_verified_report(stop_report(previous))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', help='Optional HID device path')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('identify', help='Run the identification exchange verified on this board')
    jog = sub.add_parser('jog', help='Experimental jog packet; previews by default')
    jog.add_argument('--axis', choices=['X', 'Y', 'Z', 'A', 'B'], default='X')
    jog.add_argument('--steps', type=int, required=True)
    jog.add_argument('--rate', type=int, default=100, help='Candidate steps/second')
    jog.add_argument('--acceleration', type=int, default=1000, help='Candidate steps/second squared')
    jog.add_argument('--allow-write', action='store_true')
    jog.add_argument('--seconds', type=float, help='Send JogOff after this duration (0..30 seconds)')
    jog.add_argument('--capture', type=Path, help='Save received reports as JSONL; requires --seconds')
    metric = sub.add_parser('jog-mm', help='Experimental metric jog fields; distance does not guarantee completion')
    metric.add_argument('--axis', choices=['X', 'Y', 'Z'], default='X')
    metric.add_argument('--distance', type=float, required=True, help='Signed travel field in mm; completion unverified')
    metric.add_argument('--steps-per-mm', type=float, help='Steps/mm; defaults to profile axis setting')
    metric.add_argument('--feed', type=float, help='Feed in mm/min; defaults to profile axis setting')
    metric.add_argument('--acceleration', type=float, help='Acceleration in mm/s²; defaults to profile axis setting')
    metric.add_argument('--profile', type=Path, dest='poll_profile', help='Machine settings and completion polling')
    metric.add_argument('--seconds', type=float, required=True, help='Send JogOff after this duration (0..30 seconds)')
    metric.add_argument('--allow-write', action='store_true')
    metric.add_argument('--capture', type=Path, help='Save received reports as JSONL')
    for command in (jog, metric):
        command.add_argument('--poll-profile', type=Path, help='Explicit machine profile enabling status polls; requires --seconds')
    init = sub.add_parser('initialize', help='Preview profile identification, ACK, tuning, mapping and status sequence')
    init.add_argument('--profile', type=Path, required=True)
    init.add_argument('--allow-write', action='store_true')
    interactive = sub.add_parser('console', help='Initialize once and accept manual jog/status/work-zero commands')
    interactive.add_argument('--profile', type=Path, required=True)
    interactive.add_argument('--allow-write', action='store_true', help='Required: initialization and polling change outputs')
    status = sub.add_parser('status', help='Preview output-bearing 0x03 poll using an explicit template')
    status.add_argument('--report', help='64-byte output/status template from Mach3, NOT a jog report')
    status.add_argument('--outputs-mask', type=lambda s: int(s, 0), help='Explicit polarity-adjusted Mach3 output mask (bits 0..29)')
    status.add_argument('--z-steps-per-unit', type=int, help='Explicit Z calibration for generated poll (u16)')
    status.add_argument('--allow-write', action='store_true', help='Transmit template including output/spindle settings')
    stop = sub.add_parser('stop', help='Candidate 0x0c JogOff; previews by default')
    stop.add_argument('--allow-write', action='store_true')
    stop.add_argument('--report', help='Previous 64-byte jog payload as hex; overrides saved report')
    sub.add_parser('list', help='Identify the board without sending application commands')
    monitor = sub.add_parser('monitor', help='Read unsolicited reports as JSON lines')
    monitor.add_argument('--seconds', type=float, default=10)
    send = sub.add_parser('send', help='Send one already decoded 64-byte report')
    send.add_argument('hex_payload')
    send.add_argument('--allow-write', action='store_true', help='Explicitly enable transmission')
    args = parser.parse_args()
    board = None
    try:
        if args.command == 'list':
            print(json.dumps(devices(), indent=2))
            return
        payload = None
        polling = None
        jog_identification = None
        if args.command in ('initialize', 'console'):
            setup, polling = load_profile(args.profile)
            if not args.allow_write:
                if args.command == 'console':
                    raise ValueError('console requires --allow-write')
                print(json.dumps({'sent': False, 'sequence': [p.hex() for p in setup + [polling]]}, indent=2))
                return
            board = Controller(args.path)
            print(json.dumps(initialize_board(board, setup, polling), indent=2))
            if args.command == 'console':
                console(board, json.loads(args.profile.read_text()), polling)
            return
        if args.command == 'status':
            if args.report is not None:
                if args.outputs_mask is not None or args.z_steps_per_unit is not None:
                    raise ValueError('Use --report OR --outputs-mask with --z-steps-per-unit')
                payload = status_report(bytes.fromhex(args.report))
            else:
                if args.outputs_mask is None or args.z_steps_per_unit is None:
                    raise ValueError('status requires --report OR both --outputs-mask and --z-steps-per-unit')
                payload = poll_report(args.outputs_mask, args.z_steps_per_unit)
            if not args.allow_write:
                print(json.dumps({'sent': False, 'output_settings_included': True,
                                  'hex_payload': payload.hex()}, indent=2))
                return
        if args.command == 'stop':
            previous = None
            source = 'zero buffer (no saved jog); hardware behavior unverified'
            if args.report:
                previous = bytes.fromhex(args.report)
                source = 'explicit report'
            elif LAST_JOG_FILE.exists():
                previous = bytes.fromhex(LAST_JOG_FILE.read_text())
                source = 'saved jog report'
            payload = stop_report(previous)
            if not args.allow_write:
                print(json.dumps({'experimental': True, 'sent': False,
                                  'source': source, 'hex_payload': payload.hex()}, indent=2))
                return
        if args.command in ('jog', 'jog-mm'):
            if args.poll_profile is not None:
                if args.seconds is None:
                    raise ValueError('--poll-profile requires --seconds')
                setup, polling = load_profile(args.poll_profile)
                jog_identification = setup[0]
            if args.capture is not None and args.seconds is None:
                raise ValueError('--capture requires --seconds')
            if args.seconds is not None and (not math.isfinite(args.seconds) or not 0 < args.seconds <= 30):
                raise ValueError('seconds must be finite, greater than zero and at most 30')
            fields = {}
            if args.command == 'jog-mm':
                if args.poll_profile is not None:
                    axis_settings = json.loads(args.poll_profile.read_text())['axes']['XYZ'.index(args.axis)]
                    for argument, key in (('steps_per_mm', 'steps_per_unit'),
                                          ('feed', 'feed_units_min'),
                                          ('acceleration', 'acceleration_units_s2')):
                        if getattr(args, argument) is None:
                            setattr(args, argument, axis_settings[key])
                if any(value is None for value in (args.steps_per_mm, args.feed, args.acceleration)):
                    raise ValueError('Provide --profile or all of --steps-per-mm, --feed and --acceleration')
                payload, fields = metric_jog_report('XYZAB'.index(args.axis), args.distance,
                    args.steps_per_mm, args.feed, args.acceleration)
            else:
                payload = jog_report('XYZAB'.index(args.axis), args.steps,
                                     args.rate, args.acceleration)
            if not args.allow_write:
                print(json.dumps({'experimental': True, 'sent': False,
                                  'automatic_distance_completion_verified': False,
                                  **fields, 'stop_after_seconds': args.seconds,
                                  'poll_payload': polling.hex() if polling is not None else None,
                                  'hex_payload': payload.hex()}, indent=2))
                return
        if args.command == 'send':
            if not args.allow_write:
                parser.error('send requires --allow-write; packets can activate motion or outputs')
            payload = bytes.fromhex(args.hex_payload)
            if len(payload) != REPORT_SIZE:
                parser.error('payload must contain exactly 64 bytes')
        board = Controller(args.path)
        if args.command == 'identify':
            print(json.dumps(board.identify(), indent=2))
        elif payload is not None:
            if args.command in ('jog', 'jog-mm'):
                identity = board.identify(jog_identification) if jog_identification is not None else board.identify()
                if identity['firmware_code'] != 21:
                    raise RuntimeError('Experimental jog is restricted to tested firmware code 21')
                # Save before transmission so a failed/partial write still has a stop template.
                LAST_JOG_FILE.write_text(payload.hex() + '\n')
            if args.command == 'status':
                print(json.dumps(exchange_status(board, payload), indent=2))
            elif args.command == 'stop':
                # Do not run identification or startup commands before JogOff.
                board.stop(previous)
                print(f'0x0c stop report transmitted using {source}; stopping is not confirmed.')
            elif args.command in ('jog', 'jog-mm') and args.seconds is not None:
                with (args.capture.open('x') if args.capture else nullcontext(None)) as capture:
                    if polling is not None:
                        completed = polled_jog(board, payload, args.seconds, polling, capture)
                    else:
                        timed_jog(board, payload, args.seconds, capture)
                if polling is not None:
                    print('Requested counter target reached and board ready; JogOff transmitted.' if completed else
                          'Jog timed out before counter completion; JogOff transmitted.')
                else:
                    print('Jog and timed JogOff transmitted; actual distance and completion are unconfirmed.')
            else:
                board.send_verified_report(payload)
                print('Report transmitted; this does not confirm motion or completion.')
        else:
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                report = board.read()
                if report:
                    print(json.dumps({'time_ns': time.time_ns(), 'hex': report.hex()}), flush=True)
    except (OSError, RuntimeError, ValueError, ImportError, KeyError, TypeError, OverflowError) as exc:
        parser.exit(1, f'{exc}\n')
    except KeyboardInterrupt:
        pass
    finally:
        if board is not None:
            board.close()


if __name__ == '__main__':
    main()
