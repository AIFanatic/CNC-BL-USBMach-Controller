import contextlib
import io
from pathlib import Path
import struct
import unittest
from unittest.mock import MagicMock, patch

from src import controller


class PollingTests(unittest.TestCase):
    def test_tuning_uses_each_axis_and_zeroes_reserved_bytes(self):
        axes = [{'steps_per_unit': 100 * (i + 1), 'feed_units_min': 60,
                 'acceleration_units_s2': 10} for i in range(5)]
        report = controller.tuning_report(axes, [1, 2])
        self.assertEqual(struct.unpack_from('<5H', report), (100, 200, 300, 400, 500))
        self.assertEqual(struct.unpack_from('<5I', report, 10), (100, 200, 300, 400, 500))
        self.assertEqual(struct.unpack_from('<5I', report, 30), (1000, 2000, 3000, 4000, 5000))
        self.assertEqual(report[50:], bytes([1, 2]) + bytes(11) + b'\x0e')

    def test_status_skips_wrong_command_reply(self):
        board = MagicMock()
        board.read.side_effect = [bytes(63) + b'\x62', bytes(63) + b'\x03']
        result = controller.exchange_status(board, controller.poll_report(0, 320))
        self.assertEqual(result['command_byte'], 3)
        board.send_verified_report.assert_called_once()
        self.assertEqual(board.read.call_count, 2)

    def test_poll_timeout_stops_started_jog(self):
        board = MagicMock()
        jog = controller.jog_report(0, 10, 100, 1000)
        with patch.object(controller, 'exchange_status', side_effect=[{'axis_counters': {'X': 0}}, RuntimeError('timeout')]):
            with self.assertRaises(RuntimeError):
                controller.polled_jog(board, jog, 1, controller.poll_report(0, 320))
        board.send_verified_report.assert_called_once_with(jog)
        board.stop.assert_called_once_with(jog)

    def test_completion_requires_target_and_ready_in_both_directions(self):
        for steps in (640, -640):
            with self.subTest(steps=steps):
                board = MagicMock()
                jog = controller.jog_report(0, steps, 640, 6400)
                target = 1920 + steps
                replies = [
                    {'axis_counters': {'X': 1920}},
                    {'axis_counters': {'X': 1920}, 'dll_ready_bit': True},
                    {'axis_counters': {'X': target}, 'dll_ready_bit': False},
                    {'axis_counters': {'X': target}, 'dll_ready_bit': True},
                ]
                with patch.object(controller, 'exchange_status', side_effect=replies) as exchange, \
                     patch.object(controller.time, 'sleep'):
                    self.assertTrue(controller.polled_jog(board, jog, 3, controller.poll_report(0, 640)))
                self.assertEqual(exchange.call_count, 4)
                board.send_verified_report.assert_called_once_with(jog)
                board.stop.assert_called_once_with(jog)

    def test_deadline_without_completion_returns_false_and_stops(self):
        board = MagicMock()
        jog = controller.jog_report(0, 640, 640, 6400)
        with patch.object(controller, 'exchange_status', return_value={'axis_counters': {'X': 0}}), \
             patch.object(controller.time, 'monotonic', side_effect=[0, 4]):
            self.assertFalse(controller.polled_jog(board, jog, 3, controller.poll_report(0, 640)))
        board.stop.assert_called_once_with(jog)

    def test_failed_preflight_never_starts_jog(self):
        board = MagicMock()
        with patch.object(controller, 'exchange_status', side_effect=RuntimeError('timeout')):
            with self.assertRaises(RuntimeError):
                controller.polled_jog(board, controller.jog_report(0, 10, 100, 1000),
                                      1, controller.poll_report(0, 320))
        board.send_verified_report.assert_not_called()

    def test_initialization_preview_never_opens_usb(self):
        profile = Path(__file__).resolve().parents[1] / 'machine.json'
        with patch('sys.argv', ['controller.py', 'initialize', '--profile', str(profile)]), \
             patch.object(controller, 'Controller') as factory, contextlib.redirect_stdout(io.StringIO()):
            controller.main()
        factory.assert_not_called()

    def test_initialize_transmits_order_and_checks_firmware(self):
        profile = Path(__file__).resolve().parents[1] / 'machine.json'
        setup, poll = controller.load_profile(profile)
        with patch('sys.argv', ['controller.py', 'initialize', '--profile', str(profile), '--allow-write']), \
             patch.object(controller, 'Controller') as factory, patch.object(controller.time, 'sleep'), \
             patch.object(controller, 'exchange_status', return_value={}) as exchange, \
             contextlib.redirect_stdout(io.StringIO()):
            board = factory.return_value
            board.identify.return_value = {'firmware_code': 21}
            controller.main()
        board.identify.assert_called_once_with(setup[0])
        self.assertEqual([c.args[0] for c in board.send_verified_report.call_args_list], setup[1:])
        exchange.assert_called_once_with(board, poll)
        board.close.assert_called_once()
