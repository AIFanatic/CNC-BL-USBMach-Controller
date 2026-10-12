"""Experimental coordinated XYZ line moves for plain firmware 21.

Relative millimetres, rest-to-rest trapezoidal speed, 4 ms slices.
Preview by default. This is a board motion interface, not a G-code interpreter.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from contextlib import nullcontext

from .controller import (Controller, MAX_MOVE_COUNTS, MAX_RATE, MAX_ACCELERATION,
                        capture_report, exchange_status, initialize_board, load_profile)
from .trajectory import SLICE_SECONDS, motion_record, motion_report


@dataclass
class LinePlan:
    counts: tuple
    distance_mm: tuple
    feed_mm_min: float
    acceleration_mm_s2: float
    duration_seconds: float
    records: list

    def summary(self):
        return {'delta_counts': dict(zip('XYZ', self.counts)),
                'distance_mm': dict(zip('XYZ', self.distance_mm)),
                'feed_mm_min': self.feed_mm_min,
                'acceleration_mm_s2': self.acceleration_mm_s2,
                'duration_seconds': self.duration_seconds,
                'slices': len(self.records)}


def planner_limits(profile):
    """Coordinated motion bounds; old profiles retain the existing defaults."""
    defaults = dict(max_rate_counts_s=MAX_RATE, max_acceleration_counts_s2=MAX_ACCELERATION,
                    max_move_counts=MAX_MOVE_COUNTS, max_line_seconds=30)
    settings = profile.get('planner_limits', {})
    if not isinstance(settings, dict) or settings.keys() - defaults.keys():
        raise ValueError('Unknown or invalid planner_limits settings')
    limits = defaults | settings
    for key, value in limits.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'planner_limits.{key} must be finite and positive')
    return limits


def linear_limits(profile, x=0, y=0, z=0, feed=None):
    """Quantize travel and project profile/count bounds onto a straight path."""
    limits = planner_limits(profile)
    values = (x, y, z)
    if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
        raise ValueError('Distances must be finite numbers')
    axes = profile['axes'][:3]
    if len(axes) != 3:
        raise ValueError('Profile must contain XYZ settings')
    for axis in axes:
        if not all(type(axis[key]) in (int, float) and math.isfinite(axis[key]) and axis[key] > 0
                   for key in ('steps_per_unit', 'feed_units_min', 'acceleration_units_s2')):
            raise ValueError('Axis calibration, feed and acceleration must be finite and positive')
    scales = [axis['steps_per_unit'] for axis in axes]
    scaled = [value * scale for value, scale in zip(values, scales)]
    if not all(math.isfinite(value) and abs(value) <= limits['max_move_counts'] for value in scaled):
        raise ValueError(f"Each axis is limited to {limits['max_move_counts']:g} counts per line (planner_limits.max_move_counts)")
    counts = tuple(round(value) for value in scaled)
    distances = tuple(count / scale for count, scale in zip(counts, scales))
    length = math.hypot(*distances)
    if not length:
        raise ValueError('Move rounds to zero counts')
    active = [(axis, scale, abs(distance) / length)
              for axis, scale, distance in zip(axes, scales, distances) if distance]
    if feed is None:
        feed = min(axis['feed_units_min'] for axis, _, _ in active)
    if type(feed) not in (int, float) or not math.isfinite(feed) or feed <= 0:
        raise ValueError('Feed must be finite and positive')
    # F is speed along the path; projected speed/acceleration must fit every axis.
    speed = min(feed / 60, *(min(axis['feed_units_min'] / 60, limits['max_rate_counts_s'] / scale) / fraction
                            for axis, scale, fraction in active))
    acceleration = min(min(axis['acceleration_units_s2'], limits['max_acceleration_counts_s2'] / scale) / fraction
                       for axis, scale, fraction in active)
    if not math.isfinite(speed) or not math.isfinite(acceleration) or speed <= 0 or acceleration <= 0:
        raise ValueError('Feed or acceleration cannot be represented')
    return counts, distances, speed, acceleration


def plan_linear(profile, x=0, y=0, z=0, feed=None, line_id=1):
    """Plan one relative, straight XYZ move, with integer pulse quantization."""
    counts, distances, speed, acceleration = linear_limits(profile, x, y, z, feed)
    length = math.hypot(*distances)
    ramp_time = min(speed / acceleration, math.sqrt(length / acceleration))
    peak_speed = acceleration * ramp_time
    if peak_speed <= 0:
        raise ValueError('Feed or acceleration is too small to plan')
    cruise_time = max(0, (length - peak_speed * ramp_time) / peak_speed)
    duration = 2 * ramp_time + cruise_time
    if not math.isfinite(duration) or duration > planner_limits(profile)['max_line_seconds']:
        raise ValueError(f"Planned line exceeds planner_limits.max_line_seconds ({planner_limits(profile)['max_line_seconds']:g} s)")
    records = []
    previous = (0, 0, 0)
    for sample in range(1, math.ceil(duration / SLICE_SECONDS) + 1):
        t = min(sample * SLICE_SECONDS, duration)
        if t < ramp_time:
            progress = .5 * acceleration * t * t
        elif t < ramp_time + cruise_time:
            progress = .5 * peak_speed * ramp_time + peak_speed * (t - ramp_time)
        else:
            progress = length - .5 * acceleration * (duration - t)**2
        position = tuple(round(count * min(1, max(0, progress / length))) for count in counts)
        delta = [current - old for current, old in zip(position, previous)]
        records.append(motion_record(delta + [0, 0], line_id))
        previous = position
    return LinePlan(counts, distances, peak_speed * 60, acceleration,
                    len(records) * SLICE_SECONDS, records)


def receive_reply(board, deadline, capture, phase, command):
    """Consume one matching handshake, preserving all inbound reports."""
    while (remaining := deadline - time.monotonic()) > 0:
        reply = board.read(max(1, min(50, math.ceil(remaining * 1000))))
        capture_report(capture, reply, phase)
        if len(reply) == 64 and reply[63] == command:
            return reply
    raise RuntimeError(f'No {phase} reply before deadline')


def execute_line(board, plan, polling, capture=None):
    """Stream one preplanned line, then require exact counters and readiness.

    Command 0x63 is attempted on failures/interruption. One Ctrl-C hardware test
    verified cancellation; an attempted transmission alone is not confirmation.
    """
    initial = exchange_status(board, polling, capture=capture)
    if not initial['dll_ready_bit']:
        raise RuntimeError('Board not ready; line not started')
    target = {axis: (initial['axis_counters'][axis] + delta) & 0xffffffff
              for axis, delta in zip('XYZAB', (*plan.counts, 0, 0))}
    threshold, packets_in_batch, handshakes = 12, 0, []
    start = time.monotonic()
    deadline = start + plan.duration_seconds + 3
    started = False
    try:
        started = True
        full_records = len(plan.records) // 6 * 6
        for offset in range(0, full_records, 6):
            if time.monotonic() >= deadline:
                raise RuntimeError('Line streaming deadline exceeded')
            packets_in_batch += 1
            command = 5 if packets_in_batch == threshold else 4
            board.send_verified_report(motion_report(plan.records[offset:offset+6], command))
            if command == 5:
                reply = receive_reply(board, min(deadline, time.monotonic() + .5),
                                      capture, 'buffer-handshake', 5)
                threshold = reply[0] // 10
                handshakes.append(threshold)
                packets_in_batch = 0
                if threshold == 0 and offset + 6 < len(plan.records):
                    raise RuntimeError('Buffer reply grants no next batch; streaming stopped')
        remaining = plan.records[full_records:]
        board.send_verified_report(motion_report(remaining or [bytes(10)], command=9))
        receive_reply(board, deadline, capture, 'end-of-stream', 0)
        while (remaining_time := deadline - time.monotonic()) > 0:
            status = exchange_status(board, polling, min(200, max(1, math.ceil(remaining_time * 1000))), capture)
            if status['dll_ready_bit'] and all(
                    (status['axis_counters'][axis] & 0xffffffff) == count for axis, count in target.items()):
                return {'counter_target_reached': True, 'status': status,
                        'elapsed_seconds': time.monotonic() - start,
                        'batch_thresholds': handshakes, 'physical_path_verified': False}
            time.sleep(min(.05, max(0, deadline - time.monotonic())))
        raise RuntimeError('Line did not reach its counter target and ready state')
    except BaseException as exc:
        if started:
            try:
                board.send_verified_report(bytes(63) + b'\x63')
            except Exception as reset_error:
                exc.add_note(f'Reset transmission also failed: {reset_error}')
        raise


def move_linear(board, profile, polling, x=0, y=0, z=0, feed=None, line_id=1, capture=None):
    """Relative XYZ move for an interpreter; board must be initialized."""
    return execute_line(board, plan_linear(profile, x, y, z, feed, line_id), polling, capture)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    for axis in 'xyz':
        parser.add_argument('--' + axis, type=float, default=0, help='Relative travel in mm')
    parser.add_argument('--feed', type=float, help='Path feed in mm/min; default from profile')
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--allow-write', action='store_true')
    args = parser.parse_args()
    board = None
    try:
        profile = json.loads(args.profile.read_text())
        setup, polling = load_profile(args.profile)
        plan = plan_linear(profile, args.x, args.y, args.z, args.feed)
        if not args.allow_write:
            print(json.dumps({'sent': False, **plan.summary()}, indent=2))
            return
        with (args.capture.open('x') if args.capture else nullcontext(None)) as capture:
            board = Controller()
            initialize_board(board, setup, polling)
            print(json.dumps({'plan': plan.summary(), 'result': execute_line(board, plan, polling, capture)}, indent=2))
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, OverflowError) as exc:
        parser.exit(1, f'{exc}\n')
    except KeyboardInterrupt:
        print('\nInterrupted; reset attempted if the line had started.')
    finally:
        if board is not None:
            board.close()


if __name__ == '__main__':
    main()
