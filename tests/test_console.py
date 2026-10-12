import contextlib
import io
import unittest
from unittest.mock import MagicMock, patch

from src import controller


class ConsoleTests(unittest.TestCase):
    def test_zero_is_local_and_jog_uses_axis_profile(self):
        board = MagicMock()
        profile = {'axes': [{'steps_per_unit': 818, 'feed_units_min': 60,
                             'acceleration_units_s2': 10}] * 3}
        output = io.StringIO()
        with patch('builtins.input', side_effect=['zero X', 'status', 'jog Y -10', 'quit']), \
             patch.object(controller, 'exchange_status', side_effect=[
                 {'axis_counters': {'X': 818, 'Y': 0, 'Z': 0}},
                 {'axis_counters': {'X': 1636, 'Y': 0, 'Z': 0}}]), \
             patch.object(controller, 'polled_jog', return_value=True) as jog, \
             patch.object(controller, 'LAST_JOG_FILE') as saved, \
             contextlib.redirect_stdout(output):
            controller.console(board, profile, b'poll')
        payload, _ = controller.metric_jog_report(1, -10, 818, 60, 10)
        jog.assert_called_once_with(board, payload, 15, b'poll')
        saved.write_text.assert_called_once()
        board.send_verified_report.assert_not_called()
        board.stop.assert_called_once_with(payload)
        self.assertIn('X=1.000', output.getvalue())

    def test_interrupt_during_jog_returns_to_prompt_and_stops(self):
        board = MagicMock()
        profile = {'axes': [{'steps_per_unit': 818, 'feed_units_min': 60,
                             'acceleration_units_s2': 10}] * 3}
        with patch('builtins.input', side_effect=['jog X 1', 'quit']), \
             patch.object(controller, 'polled_jog', side_effect=KeyboardInterrupt), \
             patch.object(controller, 'LAST_JOG_FILE'), \
             contextlib.redirect_stdout(io.StringIO()):
            controller.console(board, profile, b'poll')
        self.assertEqual(board.stop.call_count, 2)
