import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

from src import motion


PROFILE = {'axes': [{'steps_per_unit': 818, 'feed_units_min': 60,
                     'acceleration_units_s2': 10} for _ in range(5)]}


def counters(x=0, y=0, z=0, ready=True):
    return {'axis_counters': dict(X=x, Y=y, Z=z, A=0, B=0), 'dll_ready_bit': ready}


def increments(record):
    return [record[i + 1] * (1 if record[0] & (1 << i) else -1) for i in range(3)]


class PlannerTests(unittest.TestCase):
    def test_diagonal_endpoint_and_line_error(self):
        plan = motion.plan_linear(PROFILE, x=1, y=-.5)
        position = [0, 0, 0]
        for record in plan.records:
            for i, delta in enumerate(increments(record)):
                position[i] += delta
            self.assertLessEqual(abs(position[0] + 2 * position[1]), 1)
        self.assertEqual(position, [818, -409, 0])
        self.assertAlmostEqual(plan.duration_seconds, 1.208)

    def test_acceleration_ramps_at_both_ends(self):
        plan = motion.plan_linear(PROFILE, x=1)
        x = [increments(record)[0] for record in plan.records]
        self.assertLess(sum(x[:10]), sum(x[30:40]))
        self.assertLess(sum(x[-10:]), sum(x[30:40]))
        self.assertEqual(sum(x), 818)
        self.assertLessEqual(max(x), 4)

    def test_short_move_is_triangular_and_respects_axis_limits(self):
        plan = motion.plan_linear(PROFILE, x=.001)
        self.assertEqual(plan.counts, (1, 0, 0))
        self.assertLess(plan.feed_mm_min, 60)
        faster = motion.plan_linear(PROFILE, x=1, feed=1000)
        self.assertLessEqual(faster.feed_mm_min, 60)

    def test_invalid_moves_rejected(self):
        for kwargs in ({'x': 0}, {'x': .00001}, {'x': float('nan')},
                       {'x': 100}, {'x': 1, 'feed': 0}, {'x': 1, 'feed': .01}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                motion.plan_linear(PROFILE, **kwargs)


class StreamingTests(unittest.TestCase):
    def test_repeated_handshakes_change_batch_sizes_and_final_partial(self):
        plan = motion.plan_linear(PROFILE, x=1, y=.5)
        board = MagicMock()
        board.read.side_effect = lambda timeout: bytes([60]) + bytes(62) + b'\x05' \
            if board.send_verified_report.call_args.args[0][63] == 5 else bytes(64)
        with patch.object(motion, 'exchange_status', side_effect=[counters(), counters(818, 409)]):
            result = motion.execute_line(board, plan, b'poll')
        commands = [c.args[0][63] for c in board.send_verified_report.call_args_list]
        handshake_indices = [i for i, command in enumerate(commands) if command == 5]
        self.assertEqual(handshake_indices, [11, 17, 23, 29, 35, 41, 47])
        self.assertEqual(result['batch_thresholds'], [6] * 7)
        end = board.send_verified_report.call_args.args[0]
        self.assertEqual(end[63], 9)
        self.assertEqual(end[62], 2)
        self.assertTrue(end[10] & 0x20)
        self.assertEqual(commands.count(0x63), 0)
        calls = board.method_calls
        for i, call in enumerate(calls):
            if call[0] == 'send_verified_report' and call.args[0][63] == 5:
                self.assertEqual(calls[i + 1][0], 'read')

    def test_zero_credit_stops_before_sending_more_records(self):
        board = MagicMock()
        board.read.return_value = bytes(63) + b'\x05'
        with patch.object(motion, 'exchange_status', return_value=counters()):
            with self.assertRaisesRegex(RuntimeError, 'no next batch'):
                motion.execute_line(board, motion.plan_linear(PROFILE, x=1), b'poll')
        self.assertEqual(board.send_verified_report.call_count, 13)
        self.assertEqual(board.send_verified_report.call_args.args[0], bytes(63) + b'\x63')

    def test_keyboard_interrupt_attempts_reset(self):
        board = MagicMock()
        board.read.side_effect = KeyboardInterrupt
        with patch.object(motion, 'exchange_status', return_value=counters()):
            with self.assertRaises(KeyboardInterrupt):
                motion.execute_line(board, motion.plan_linear(PROFILE, x=1), b'poll')
        self.assertEqual(board.send_verified_report.call_args.args[0], bytes(63) + b'\x63')

    def test_busy_preflight_does_not_start(self):
        board = MagicMock()
        with patch.object(motion, 'exchange_status', return_value=counters(ready=False)):
            with self.assertRaises(RuntimeError):
                motion.execute_line(board, motion.plan_linear(PROFILE, x=1), b'poll')
        board.send_verified_report.assert_not_called()

    def test_preview_does_not_open_usb(self):
        profile = Path(__file__).resolve().parents[1] / 'machine.json'
        with patch('sys.argv', ['motion.py', '--profile', str(profile), '--x', '1', '--y', '.5']), \
             patch.object(motion, 'Controller') as factory, contextlib.redirect_stdout(io.StringIO()) as output:
            motion.main()
        self.assertFalse(json.loads(output.getvalue())['sent'])
        factory.assert_not_called()


class ContinuousPlannerTests(unittest.TestCase):
    def compile(self, text, profile=PROFILE):
        from src.gcode_runner import compile_program
        from src.planner import combined_plan
        moves = compile_program(text, profile)
        return moves, combined_plan(moves)

    def test_straight_joins_have_one_ramp_pair_and_no_per_line_padding(self):
        moves, plan = self.compile('G1 F60\nX.2\nX.4\nX.6\nX.8\nX1')
        self.assertAlmostEqual(plan.duration_seconds, 1.1)
        self.assertEqual(sum(increments(r)[0] for r in plan.records), 818)
        # At the first line boundary, speed is maintained rather than falling to zero.
        first_id = moves[0].source_line
        boundary = next(i for i, r in enumerate(plan.records) if int.from_bytes(r[6:9], 'little') != first_id)
        self.assertGreaterEqual(sum(increments(r)[0] for r in plan.records[boundary-4:boundary+4]), 24)

    def test_short_segments_use_requested_feed_not_isolated_peak_speed(self):
        _, plan = self.compile('G91\nG1 F60\n' + 'X.001\n' * 1000)
        self.assertAlmostEqual(plan.duration_seconds, 1.1)
        self.assertEqual(plan.counts, (818, 0, 0))
        self.assertLessEqual(len(plan.records), 276)

    def test_reversals_and_corners_reach_exact_stop_vertices(self):
        for text, vertex in [('G1 X1 F60\nX0', (818, 0, 0)),
                             ('G1 X1 F60\nY1\nX0 Y0', (818, 818, 0))]:
            moves, plan = self.compile(text)
            position, visited = [0, 0, 0], []
            for record in plan.records:
                position = [p+d for p, d in zip(position, increments(record))]
                visited.append(tuple(position))
            self.assertIn(vertex, visited)
            self.assertEqual(tuple(position), plan.counts)
            for move in moves:
                self.assertLessEqual(abs(increments(move.plan.records[-1])[0]), 1)

    def test_mixed_axis_caps_and_braking_for_lower_feed(self):
        profile = {'axes': [dict(a) for a in PROFILE['axes']]}
        profile['axes'][1]['feed_units_min'] = 30
        moves, plan = self.compile('G1 X1 Y1 F600\nX1.01 Y1.01 F6', profile)
        self.assertLessEqual(moves[0].plan.feed_mm_min, 30 * 2**.5 + 1e-9)
        self.assertLessEqual(moves[1].plan.feed_mm_min, 6)
        x = [abs(increments(r)[0]) for r in moves[0].plan.records]
        self.assertLess(sum(x[-10:]), sum(x[-60:-50]))
        self.assertLessEqual(x[-1], 1)
        self.assertEqual(plan.counts, (826, 826, 0))
        self.assertTrue(all(max(map(abs, increments(r))) <= 4 for r in plan.records))

    def test_whole_closed_program_has_one_end_and_preserves_initial_counters(self):
        _, plan = self.compile('G1 X1 F60\nX0')
        board = MagicMock()
        board.read.side_effect = lambda timeout: bytes([60]) + bytes(62) + b'\x05' \
            if board.send_verified_report.call_args.args[0][63] == 5 else bytes(64)
        with patch.object(motion, 'exchange_status', side_effect=[counters(100, -100), counters(100, -100)]):
            result = motion.execute_line(board, plan, b'poll')
        packets = [c.args[0] for c in board.send_verified_report.call_args_list]
        self.assertTrue(result['counter_target_reached'])
        self.assertEqual(sum(p[63] == 9 for p in packets), 1)
        self.assertGreater(sum(p[63] == 5 for p in packets), 1)
        self.assertEqual(packets[-1][63], 9)

    def test_continuous_stream_interrupt_uses_existing_reset(self):
        _, plan = self.compile('G1 X1 F60\nX2')
        board = MagicMock()
        board.read.side_effect = KeyboardInterrupt
        with patch.object(motion, 'exchange_status', return_value=counters()):
            with self.assertRaises(KeyboardInterrupt):
                motion.execute_line(board, plan, b'poll')
        self.assertEqual(board.send_verified_report.call_args.args[0], bytes(63) + b'\x63')


class ArcPlannerTests(unittest.TestCase):
    def compile(self, text, profile=PROFILE):
        from src.gcode_runner import compile_program
        from src.planner import combined_plan
        moves = compile_program(text, profile)
        return moves, combined_plan(moves)

    def test_full_circles_have_correct_direction_radius_and_close(self):
        import math
        for mode, sign in [('G2', 1), ('G3', -1)]:
            moves, plan = self.compile(f'{mode} I2 J0 F60')
            position, first_y = [0, 0, 0], None
            for record in plan.records:
                position = [p+d for p, d in zip(position, increments(record))]
                if first_y is None and position[1]:
                    first_y = position[1]
                self.assertLessEqual(abs(math.hypot(position[0]-1636, position[1])-1636), .71)
            self.assertGreater(first_y*sign, 0)
            self.assertEqual(position, [0, 0, 0])
            self.assertEqual(plan.counts, (0, 0, 0))
            self.assertAlmostEqual(plan.duration_seconds, 12.768)
            # A curve has one acceleration/deceleration pair, not tiny stopped lines.
            self.assertEqual(len(moves), 1)
            self.assertTrue(all(any(increments(r)) for r in plan.records[100:-100]))

    def test_tangent_quarter_arcs_keep_speed_through_the_join(self):
        moves, plan = self.compile('G1 X2 F60\nG3 X4 Y2 I0 J2\nG3 X2 Y4 I-2 J0')
        self.assertLessEqual(abs(increments(moves[1].plan.records[-1])[0]), 1)
        self.assertGreaterEqual(abs(increments(moves[1].plan.records[-1])[1]), 3)
        self.assertGreaterEqual(abs(increments(moves[2].plan.records[0])[1]), 3)
        self.assertEqual(plan.counts, (1636, 3272, 0))

    def test_curvature_reserves_acceleration_budget(self):
        import math
        moves, plan = self.compile('G3 I.02 J0 F600')
        move = moves[0]
        speed = move.plan.feed_mm_min/60
        self.assertLessEqual(speed*speed/move.arc.radius + move.plan.acceleration_mm_s2, 10+1e-10)
        self.assertLess(move.plan.feed_mm_min, 60)
        self.assertEqual(plan.counts, (0, 0, 0))

    def test_minor_major_and_clockwise_radius_formats(self):
        import math
        for mode in ('G2', 'G3'):
            for radius, expected in [(2, math.pi/2), (-2, 3*math.pi/2)]:
                moves, plan = self.compile(f'{mode} X2 Y2 R{radius} F60')
                self.assertAlmostEqual(abs(moves[0].arc.sweep), expected)
                self.assertEqual(moves[0].arc.sweep > 0, mode == 'G3')
                self.assertEqual(plan.counts, (1636, 1636, 0))

    def test_incremental_endpoints_and_nonmodal_centers(self):
        moves, plan = self.compile('G17 G91 G91.1\nG3 X2 Y2 I0 J2 F60\nX-2 Y2 I-2 J0')
        self.assertEqual(plan.counts, (0, 3272, 0))
        with self.assertRaisesRegex(ValueError, 'Line 2:'):
            self.compile('G3 X2 Y2 I0 J2 F60\nX0 Y4')

    def test_invalid_arcs_and_unsupported_planes_are_rejected(self):
        for text in ('G3 I0 J0 F60', 'G3 X1 Y1 I0 J2 F60', 'G2 X2 R.1 F60',
                     'G3 R2 F60', 'G3 I2 R2 F60', 'G3 I2 J0 Z1 F60',
                     'G18', 'G90.1', 'G3 I2 K1 F60', 'G3 I2 P2 F60',
                     'G3 I7 F60', 'G3 I5 F60', 'G1 X1 I1 F60', 'G3 X.0001 I2 F60'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.compile(text)

    def test_preview_contains_curve_instead_of_zero_length_chord(self):
        moves, _ = self.compile('G3 I2 F60')
        path = moves[0].summary()['path_xy']
        self.assertAlmostEqual(max(p[0] for p in path), 4)
        self.assertAlmostEqual(min(p[1] for p in path), -2)
        self.assertAlmostEqual(max(p[1] for p in path), 2)
        self.assertEqual(path[-1], (0, 0))

    def test_full_circle_stream_has_one_end_and_uses_repeated_handshakes(self):
        _, plan = self.compile('G3 I2 F60')
        board = MagicMock()
        board.read.side_effect = lambda timeout: bytes([60]) + bytes(62) + b'\x05' \
            if board.send_verified_report.call_args.args[0][63] == 5 else bytes(64)
        with patch.object(motion, 'exchange_status', side_effect=[counters(100, -50), counters(100, -50)]):
            result = motion.execute_line(board, plan, b'poll')
        commands = [call.args[0][63] for call in board.send_verified_report.call_args_list]
        self.assertTrue(result['counter_target_reached'])
        self.assertEqual(commands.count(9), 1)
        self.assertGreater(commands.count(5), 50)


class ProfileLimitTests(unittest.TestCase):
    def test_duration_setting_allows_ten_mm_full_circle(self):
        from src.gcode_runner import compile_program
        profile = {**PROFILE, 'planner_limits': {'max_line_seconds': 60}}
        moves = compile_program('G3 I5 F60', profile)
        self.assertGreater(moves[0].plan.duration_seconds, 30)
        with self.assertRaisesRegex(ValueError, 'max_line_seconds'):
            compile_program('G3 I5 F60', PROFILE)

    def test_travel_rate_and_acceleration_settings_are_applied(self):
        profile = {**PROFILE, 'planner_limits': {'max_move_counts': 20000}}
        self.assertEqual(motion.plan_linear(profile, x=20).counts[0], 16360)
        profile = {**PROFILE, 'planner_limits': {'max_rate_counts_s': 500, 'max_acceleration_counts_s2': 1000}}
        plan = motion.plan_linear(profile, x=1)
        self.assertLessEqual(plan.feed_mm_min, 500/818*60)
        self.assertLessEqual(plan.acceleration_mm_s2, 1000/818)

    def test_arc_excursion_uses_profile_setting(self):
        from src.gcode_runner import compile_program
        profile = {**PROFILE, 'planner_limits': {'max_move_counts': 20000, 'max_line_seconds': 60}}
        self.assertTrue(compile_program('G3 I7 F60', profile))
        with self.assertRaisesRegex(ValueError, 'max_move_counts'):
            compile_program('G3 I7 F60', PROFILE)

    def test_invalid_limits_are_rejected(self):
        for setting in ({'max_line_seconds': 0}, {'max_move_counts': -1},
                        {'max_rate_counts_s': float('nan')}, {'max_line_seconds': True},
                        {'max_line_seconds': '60'}, {'unknown': 1}):
            with self.subTest(setting=setting), self.assertRaises(ValueError):
                motion.plan_linear({**PROFILE, 'planner_limits': setting}, x=1)
