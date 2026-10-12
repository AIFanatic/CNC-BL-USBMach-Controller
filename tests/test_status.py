import io
import struct
import unittest
from unittest.mock import MagicMock, patch

from src import controller


class StatusTests(unittest.TestCase):
    def test_generated_poll_layout(self):
        report = controller.poll_report(0x12345678, 320, 50, True, False, True, .25, True)
        expected = bytearray(64)
        struct.pack_into('<IBBH', expected, 0, 0x12345678, 50, 1, 320)
        expected[14] = expected[19] = expected[24] = expected[26] = 1
        struct.pack_into('<f', expected, 15, .25)
        struct.pack_into('<f', expected, 20, .25)
        expected[63] = 3
        self.assertEqual(report, bytes(expected))

    def test_estop_inhibits_spindle_and_bounds_rejected(self):
        report = controller.poll_report(0, 320, 50, True, True)
        self.assertEqual((report[5], report[8]), (0, 1))
        for args in [(-1, 320), (1 << 30, 320), (0, 65536), (0, 320, 101)]:
            with self.assertRaises(ValueError):
                controller.poll_report(*args)

    def test_template_retains_output_settings(self):
        template = bytes(range(64))
        payload = controller.status_report(template)
        self.assertEqual(payload[:63], template[:63])
        self.assertEqual(payload[63], 3)
        with self.assertRaises(ValueError):
            controller.status_report(bytes(63))

    def test_candidate_counters_are_signed_and_flags_independent(self):
        report = bytearray(64)
        struct.pack_into('<5i', report, 4, -1, 2, -3, 4, -5)
        report[1], report[40] = 0x80, 0x81
        decoded = controller.decode_status(bytes(report))
        self.assertEqual(decoded['axis_counters'], dict(zip('XYZAB', [-1, 2, -3, 4, -5])))
        self.assertTrue(decoded['dll_ready_bit'])
        self.assertEqual(decoded['flags_offset_40'], 0x81)

    def test_capture_reads_without_resending_jog_or_poll(self):
        board = MagicMock()
        board.read.return_value = b'\x01\x02'
        payload = controller.jog_report(0, 10, 100, 1000)
        capture = io.StringIO()
        with patch.object(controller.time, 'monotonic', side_effect=[0, .01, .06]):
            controller.timed_jog(board, payload, .05, capture)
        board.send_verified_report.assert_called_once_with(payload)
        board.read.assert_called_once_with(40)
        board.stop.assert_called_once_with(payload)
        self.assertIn('0102', capture.getvalue())

    def test_read_failure_still_stops(self):
        board = MagicMock()
        board.read.side_effect = OSError('USB read failed')
        payload = controller.jog_report(0, 10, 100, 1000)
        with patch.object(controller.time, 'monotonic', side_effect=[0, .01]):
            with self.assertRaises(OSError):
                controller.timed_jog(board, payload, 1, io.StringIO())
        board.stop.assert_called_once_with(payload)
