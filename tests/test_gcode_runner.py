import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from src.gcode_runner import compile_program, main

PROFILE = {'axes': [{'steps_per_unit': 818, 'feed_units_min': 60,
                     'acceleration_units_s2': 10} for _ in range(5)]}
ROOT = Path(__file__).resolve().parents[1]


class GcodeTests(unittest.TestCase):
    def test_library_handles_absolute_relative_and_modal_moves(self):
        moves = compile_program('G21 G90\nG1 X1 Y.5 F60\nY0\nG91\nX-1\nM2', PROFILE)
        self.assertEqual([move.plan.counts for move in moves], [(818, 409, 0), (0, -409, 0), (-818, 0, 0)])
        self.assertEqual(moves[-1].target_mm, (0, 0, 0))

    def test_substep_targets_accumulate_instead_of_disappearing(self):
        text = 'G21 G91\nG1 F60\n' + 'X.0004\n' * 10
        moves = compile_program(text, PROFILE)
        self.assertEqual(sum(move.plan.counts[0] for move in moves), 3)

    def test_rapid_uses_profile_feed_and_preserves_linear_feed(self):
        moves = compile_program('G1 X1 F30\nG0 Y1\nG1 X0', PROFILE)
        self.assertEqual([move.mode for move in moves], ['G1', 'G0', 'G1'])
        self.assertEqual([move.plan.feed_mm_min for move in moves], [30, 60, 30])

    def test_unsupported_commands_and_syntax_fail_with_line_number(self):
        for text in ('G2 X1 I.5', 'G20', 'M3 S1000', 'G92 X0', '/G1 X1 F60',
                     'G1 X[1+2] F60', 'G1 X1 X2 F60', 'G1 X1',
                     'G1 X1 F0', 'G1 X1 A2 F60', 'G90 G91', 'M2\nG1 X1 F60',
                     'G1 X1 F60\nG2 X0 I1', 'G1 X1 F60 M2'):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'Line [12]:'):
                compile_program(text, PROFILE)

    def test_comments_line_numbers_and_program_delimiters(self):
        moves = compile_program('%\n(comment)\nN10 G21 G90\nN20 G1 X1 F60 ; move\nN30 M30\n%', PROFILE)
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0].source_line, 4)

    def test_semicolons_inside_parenthesis_comments_and_circle_example(self):
        moves = compile_program('(comment; still a comment)\nG1 X1 F60', PROFILE)
        self.assertEqual(len(moves), 1)
        circle = compile_program((ROOT / 'research/examples/circle.ngc').read_text(), PROFILE)
        self.assertEqual(len(circle), 3)
        self.assertEqual(circle[-1].target_mm, (0, 0, 0))

    def test_late_invalid_line_prevents_any_usb_access_even_with_write_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / 'invalid.ngc'
            program.write_text('G1 X1 F60\nM3 S1000\n')
            with patch('sys.argv', ['gcode_runner.py', str(program), '--profile', str(ROOT / 'machine.json'), '--allow-write']), \
                 patch('src.gcode_runner.Controller') as factory, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main()
            factory.assert_not_called()

    def test_preview_example_never_opens_usb(self):
        with patch('sys.argv', ['gcode_runner.py', str(ROOT / 'research/examples/triangle.ngc'), '--profile', str(ROOT / 'machine.json')]), \
             patch('src.gcode_runner.Controller') as factory, contextlib.redirect_stdout(io.StringIO()) as output:
            main()
        factory.assert_not_called()
        self.assertIn('"move_count": 3', output.getvalue())

    def test_execution_initializes_once_and_streams_whole_program(self):
        with patch('sys.argv', ['gcode_runner.py', str(ROOT / 'research/examples/triangle.ngc'), '--profile', str(ROOT / 'machine.json'), '--allow-write']), \
             patch('src.gcode_runner.Controller') as factory, \
             patch('src.gcode_runner.initialize_board') as initialize, \
             patch('src.gcode_runner.execute_line', return_value={'counter_target_reached': True}) as execute, \
             contextlib.redirect_stdout(io.StringIO()):
            main()
        initialize.assert_called_once()
        self.assertEqual(execute.call_count, 1)
        factory.return_value.close.assert_called_once()

    def test_failed_line_stops_program_and_closes_board(self):
        with patch('sys.argv', ['gcode_runner.py', str(ROOT / 'research/examples/triangle.ngc'), '--profile', str(ROOT / 'machine.json'), '--allow-write']), \
             patch('src.gcode_runner.Controller') as factory, patch('src.gcode_runner.initialize_board'), \
             patch('src.gcode_runner.execute_line', side_effect=RuntimeError('stream failure')) as execute, \
             contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main()
        self.assertEqual(execute.call_count, 1)
        factory.return_value.close.assert_called_once()

    def test_lexer_rejects_garbage_and_nonfinite_values(self):
        for source in ('garbage G1 X1 F60', 'G1 X1 F60 trailing', 'G1 X1 F60 !',
                       'G1 Xnan F60', 'G1 X1e2 F60', '(unclosed G1 X1 F60',
                       'G1 X1 F60 N-1', 'G1 X1 F60 G0', 'G1 X1 F.01'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                compile_program(source, PROFILE)

    def test_feed_only_and_modes_apply_before_coordinates(self):
        moves = compile_program('F30\nX1 G1\nX1 G91\nG90 X0\nM30', PROFILE)
        self.assertEqual([m.target_mm[0] for m in moves], [1, 2, 0])
        self.assertEqual([m.requested_feed for m in moves], [30, 30, 30])
