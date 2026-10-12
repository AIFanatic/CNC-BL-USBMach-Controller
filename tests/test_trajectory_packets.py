import unittest
from src.trajectory import motion_record, motion_report, stream_progress


class TrajectoryPacketTests(unittest.TestCase):
    def test_status_line_id_and_threshold_are_distinct(self):
        report = bytearray(64)
        report[28:32] = (123).to_bytes(4, 'little')
        report[32:36] = (12).to_bytes(4, 'little')
        report[63] = 3
        self.assertEqual(stream_progress(report), {'line_id': 123, 'batch_threshold': 12})
        report[63] = 5
        with self.assertRaises(ValueError):
            stream_progress(report)

    def test_low_bytes_signs_and_little_endian_field(self):
        self.assertEqual(motion_record([1, -2, 3, -4, 0], 0x123456),
                         bytes.fromhex('05010203040056341200'))

    def test_high_bits_follow_dll_bit_order(self):
        self.assertEqual(motion_record([256, -257, 258, -259, 400], 1),
                         bytes.fromhex('55000102039001000055'))

    def test_six_records_count_and_command(self):
        record = motion_record([0] * 5, 0)
        packet = motion_report([record] * 6, command=5)
        self.assertEqual(packet, bytes(62) + bytes([6, 5]))

    def test_finish_marks_last_record_and_preserves_previous(self):
        record = motion_record([1, 0, 0, 0, 0], 1)
        packet = motion_report([record, record], command=9)
        self.assertEqual(packet[:10], record)
        self.assertEqual(packet[10], 0x21)
        self.assertEqual(packet[62:], bytes([2, 9]))

    def test_reject_unrepresentable_inputs(self):
        for axes in ([401, 0, 0, 0, 0], [1.5, 0, 0, 0, 0], [0] * 6):
            with self.assertRaises(ValueError):
                motion_record(axes, 1)
        with self.assertRaises(ValueError):
            motion_report([])
