import contextlib
import io
import struct
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from src import controller


class MetricJogTests(unittest.TestCase):
    def test_metric_conversion_and_negative_direction(self):
        payload, info = controller.metric_jog_report(1, -2, 320, 60, 10)
        self.assertEqual(struct.unpack_from('<7I', payload), (1, 0, 640, 320, 3200, 0, 0))
        self.assertEqual(info['quantized_distance_mm'], -2)

    def test_truncates_like_dll(self):
        payload, _ = controller.metric_jog_report(0, .123, 320, 60.1, 1.01)
        self.assertEqual(struct.unpack_from('<7I', payload)[2:5], (39, 320, 323))

    def test_invalid_or_unrepresentable_values(self):
        for values in [(0, 320, 60, 10), (1, 0, 60, 10), (float('nan'), 320, 60, 10),
                       (1, 320, float('inf'), 10), (.001, 320, 60, 10),
                       (1, 320, .01, 10), (1, 320, 600, 10), (1e300, 1e300, 60, 10)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                controller.metric_jog_report(0, *values)

    def test_stop_on_timeout_interrupt_and_partial_write(self):
        payload = controller.jog_report(0, 10, 100, 1000)
        for failure in (None, KeyboardInterrupt(), RuntimeError('partial write')):
            board = MagicMock()
            with patch.object(controller.time, 'sleep') as sleep:
                if isinstance(failure, KeyboardInterrupt):
                    sleep.side_effect = failure
                elif failure:
                    board.send_verified_report.side_effect = failure
                if failure:
                    with self.assertRaises(type(failure)):
                        controller.timed_jog(board, payload, .1)
                else:
                    controller.timed_jog(board, payload, .1)
                    sleep.assert_called_once_with(.1)
                board.stop.assert_called_once_with(payload)

    def test_metric_preview_does_not_open_usb(self):
        args = ['controller.py', 'jog-mm', '--distance', '1', '--steps-per-mm', '320',
                '--feed', '60', '--acceleration', '10', '--seconds', '2']
        with patch('sys.argv', args), patch.object(controller, 'Controller') as factory, \
             contextlib.redirect_stdout(io.StringIO()):
            controller.main()
            factory.assert_not_called()

    def test_cli_metric_write_identifies_saves_and_stops(self):
        args = ['controller.py', 'jog-mm', '--distance', '1', '--steps-per-mm', '320',
                '--feed', '60', '--acceleration', '10', '--seconds', '.1', '--allow-write']
        payload, _ = controller.metric_jog_report(0, 1, 320, 60, 10)
        with tempfile.TemporaryDirectory() as tmp, patch('sys.argv', args), \
             patch.object(controller, 'LAST_JOG_FILE', Path(tmp) / 'jog.hex'), \
             patch.object(controller, 'Controller') as factory, \
             patch.object(controller.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
            board = factory.return_value
            board.identify.return_value = {'firmware_code': 21}
            controller.main()
            board.identify.assert_called_once()
            board.send_verified_report.assert_called_once_with(payload)
            board.stop.assert_called_once_with(payload)
            board.close.assert_called_once()
            self.assertEqual(controller.LAST_JOG_FILE.read_text().strip(), payload.hex())


if __name__ == '__main__':
    unittest.main()
