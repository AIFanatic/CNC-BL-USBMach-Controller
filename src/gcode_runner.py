"""Limited G-code runner with continuous straight joins for BL-UsbMach.

Supports G0/G1, XY G2/G3, G17, G21, G90/G91, G91.1, G94, F, XYZ, N, M2/M30.
Coordinates are relative to the position at program start (local work zero).
No helices, spindle commands, tools, offsets, macros or corner rounding.
"""
import argparse
from contextlib import nullcontext
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

from .controller import Controller, initialize_board, load_profile
from .motion import LinePlan, execute_line, linear_limits, planner_limits
from .planner import Arc, combined_plan, make_arc, plan_program

# Full-block validation prevents accepting skipped garbage or partial words.
PLAIN_WORDS = re.compile(r'(?:\s*[A-Z][+-]?(?:\d+(?:\.\d*)?|\.\d+))*\s*', re.IGNORECASE)
WORDS = re.compile(r'([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))', re.IGNORECASE)


@dataclass
class ProgramMove:
    source_line: int
    mode: str
    target_mm: tuple
    plan: LinePlan
    requested_feed: float | None = None
    arc: Arc | None = None

    def summary(self):
        summary = {'source_line': self.source_line, 'mode': self.mode,
                'target_mm': dict(zip('XYZ', self.target_mm)), **self.plan.summary()}
        if self.arc:
            samples = max(2, math.ceil(abs(self.arc.sweep) * 90 / math.pi))
            summary['path_xy'] = [self.arc.point(i/samples)[:2] for i in range(samples+1)]
            summary['path_xy'][-1] = self.target_mm[:2]
            summary['radius_mm'] = self.arc.radius
            summary['sweep_radians'] = self.arc.sweep
        return summary


def compile_program(text, profile):
    """Validate our supported subset, interpret modes, and plan before USB use."""
    scales = [axis['steps_per_unit'] for axis in profile['axes'][:3]]
    position, actual_counts = (0., 0., 0.), (0, 0, 0)
    moves, ended, absolute, mode, feed = [], False, True, None, None
    for number, source in enumerate(text.splitlines(), 1):
        try:
            if source.strip() == '%':
                continue
            block = re.sub(r'\([^()]*\)', '', source).split(';', 1)[0].strip().upper()
            if not block:
                continue
            if not PLAIN_WORDS.fullmatch(block):
                raise ValueError('Expressions, macros and unsupported syntax')
            if ended:
                raise ValueError('Commands after M2/M30 are unsupported')
            words = [(letter, float(value)) for letter, value in WORDS.findall(block)]
            values, groups, end_words = {}, set(), []
            for letter, value in words:
                if not math.isfinite(value):
                    raise ValueError('Word values must be finite numbers')
                if letter == 'G':
                    if value not in (0, 1, 2, 3, 17, 21, 90, 91, 91.1, 94):
                        raise ValueError(f'Unsupported command G{value:g}')
                    group = 'motion' if value in (0, 1, 2, 3) else 'distance' if value in (90, 91) else str(value)
                    if group in groups:
                        raise ValueError('Conflicting or duplicate G commands')
                    groups.add(group)
                    if value in (0, 1, 2, 3):
                        mode = 'G' + str(int(value))
                    elif value in (90, 91):
                        absolute = value == 90
                elif letter == 'M':
                    if value not in (2, 30):
                        raise ValueError(f'Unsupported command M{value:g}')
                    end_words.append(value)
                elif letter not in 'XYZIJRFN':
                    raise ValueError(f'Unsupported word {letter}')
                else:
                    if letter in values:
                        raise ValueError(f'Duplicate {letter} word')
                    values[letter] = value
                if letter == 'F' and value <= 0:
                    raise ValueError('Feed must be positive')
                if letter == 'N' and (value < 0 or value != int(value)):
                    raise ValueError('N must be a nonnegative integer')
            if end_words:
                if len(end_words) != 1 or any(letter not in 'MN' for letter, _ in words):
                    raise ValueError('M2/M30 must be on a separate block')
                ended = True
                continue
            feed = values.get('F', feed)
            is_arc = mode in ('G2', 'G3')
            if not is_arc and any(word in values for word in 'IJR'):
                raise ValueError('I/J/R are only valid with G2/G3')
            if not any(axis in values for axis in 'XYZIJR'):
                continue
            if mode is None:
                raise ValueError('Select a motion command before giving coordinates')
            if mode != 'G0' and feed is None:
                raise ValueError('G1/G2/G3 require a positive F setting')
            old_position = position
            position = tuple(values.get(axis, old) if absolute else old+values.get(axis, 0.)
                             for axis, old in zip('XYZ', position))
            scaled = [p*s for p, s in zip(position, scales)]
            if not all(math.isfinite(value) for value in scaled):
                raise ValueError('Target coordinates cannot be represented')
            target_counts = tuple(round(value) for value in scaled)
            if is_arc and target_counts == actual_counts and math.dist(position[:2], old_position[:2]) > 1e-12:
                raise ValueError('Arc endpoint rounds to its start; cannot distinguish it from a full circle')
            delta = tuple((new-old)/scale for new, old, scale in zip(target_counts, actual_counts, scales))
            if target_counts == actual_counts and not is_arc:
                continue
            requested = None if mode == 'G0' else feed
            physical_target = tuple(count/scale for count, scale in zip(target_counts, scales))
            origin = tuple(count/scale for count, scale in zip(actual_counts, scales))
            arc = make_arc(origin, physical_target, values, mode == 'G2', scales, planner_limits(profile)['max_move_counts']) if is_arc else None
            # Validate profile settings even for zero-net-displacement circles.
            counts, distances, speed, acceleration = linear_limits(profile, *(delta if not arc else (1/scales[0], 0, 0)), feed=requested)
            if arc:
                counts, distances = tuple(new-old for new, old in zip(target_counts, actual_counts)), delta
            plan = LinePlan(counts, distances, speed*60, acceleration, 0., [])
            actual_counts = target_counts
            moves.append(ProgramMove(number, mode, physical_target, plan, requested, arc))
        except (ValueError, OverflowError) as exc:
            raise ValueError(f'Line {number}: {exc}') from exc
    if moves:
        plan_program(moves, profile)
    return moves


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', type=Path)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--allow-write', action='store_true')
    args = parser.parse_args()
    board = None
    try:
        profile = json.loads(args.profile.read_text())
        setup, polling = load_profile(args.profile)
        moves = compile_program(args.file.read_text(), profile)
        summary = {'sent': False, 'units': 'mm', 'work_zero': 'position at program start',
                   'interpreter': 'built-in G0/G1 subset', 'continuous_straight_joins': True, 'move_count': len(moves),
                   'nominal_seconds': sum(move.plan.duration_seconds for move in moves),
                   'moves': [move.summary() for move in moves]}
        if not args.allow_write or not moves:
            print(json.dumps(summary, indent=2))
            return
        with (args.capture.open('x') if args.capture else nullcontext(None)) as capture:
            board = Controller()
            initialize_board(board, setup, polling)
            result = execute_line(board, combined_plan(moves), polling, capture)
            print(json.dumps({'moves': len(moves), 'result': result}), flush=True)
        print(f'Completed {len(moves)} moves by reported counters; physical path remains unverified.')
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, OverflowError) as exc:
        parser.exit(1, f'{exc}\n')
    except KeyboardInterrupt:
        print('\nInterrupted; reset attempted if a line was streaming. Program will not continue.')
    finally:
        if board is not None:
            board.close()


if __name__ == '__main__':
    main()
