import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock

from src import controller


class StopTests(unittest.TestCase):
    def test_preserves_previous_payload_except_command(self):
        previous = controller.jog_report(2, -123, 80, 900)
        report = controller.stop_report(previous)
        self.assertEqual(report[:63], previous[:63])
        self.assertEqual(report[63], 0x0c)
        self.assertEqual(previous[63], 0x0b)

    def test_no_previous_report_has_only_stop_command(self):
        self.assertEqual(controller.stop_report(), bytes(63) + b'\x0c')

    def test_invalid_report_rejected(self):
        for length in [0, 63, 65]:
            with self.assertRaises(ValueError):
                controller.stop_report(bytes(length))

    def test_api_writes_report_id_prefix(self):
        board = controller.Controller.__new__(controller.Controller)
        board.device = MagicMock()
        board.device.write.return_value = 65
        board.stop()
        board.device.write.assert_called_once_with(bytes(64) + b'\x0c')

    def test_cli_preview_does_not_open_device(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / 'last.hex'
            with patch.object(controller, 'LAST_JOG_FILE', state), \
                 patch.object(controller, 'Controller') as factory, \
                 patch('sys.argv', ['controller.py', 'stop']), \
                 contextlib.redirect_stdout(io.StringIO()):
                controller.main()
                factory.assert_not_called()
                self.assertFalse(state.exists())

    def test_cli_stop_uses_saved_report_without_identification(self):
        previous = controller.jog_report(1, -55, 20, 100)
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / 'last.hex'
            state.write_text(previous.hex())
            with patch.object(controller, 'LAST_JOG_FILE', state), \
                 patch.object(controller, 'Controller') as factory, \
                 patch('sys.argv', ['controller.py', 'stop', '--allow-write']), \
                 contextlib.redirect_stdout(io.StringIO()):
                controller.main()
                board = factory.return_value
                board.identify.assert_not_called()
                board.stop.assert_called_once_with(previous)
                board.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
