import unittest
from unittest.mock import MagicMock, patch

from research.tools.coordinated_test import run_test, test_reports


def status(x, y, ready=True):
    return {'dll_ready_bit': ready, 'axis_counters': dict(X=x, Y=y, Z=0, A=0, B=0)}


class CoordinatedTestTests(unittest.TestCase):
    def test_buffer_handshake_is_read_before_more_motion(self):
        packets = test_reports(1, 1, 78)
        self.assertEqual([p[63] for p in packets], [4] * 11 + [5, 4, 9])
        board = MagicMock()
        board.read.side_effect = [bytes([120]) + bytes(63), bytes(64)]
        with patch('research.tools.coordinated_test.exchange_status', side_effect=[
                status(0, 0), *[status(i, i, False) for i in range(12)], status(78, 78)]):
            result = run_test(board, b'poll', packets, None, 1, 1, 78)
        self.assertEqual(result['buffer_handshake']['dll_next_batch_threshold'], 12)
        calls = board.method_calls
        handshake_write = next(i for i, call in enumerate(calls)
                               if call[0] == 'send_verified_report' and call.args[0][63] == 5)
        self.assertEqual(calls[handshake_write + 1][0], 'read')
        self.assertEqual(calls[handshake_write + 2].args[0][63], 4)

    def test_extended_stream_stays_below_batch_limit_and_polls_in_flight(self):
        packets = test_reports(1, -1, 60)
        self.assertEqual(len(packets), 11)
        self.assertTrue(all(p[63] == 4 and p[62] == 6 for p in packets[:-1]))
        board = MagicMock()
        board.read.return_value = bytes(64)
        with patch('research.tools.coordinated_test.exchange_status', side_effect=[
                status(0, 0), *[status(i, -i, False) for i in range(10)], status(60, -60)]) as exchange:
            result = run_test(board, b'poll', packets, None, 1, -1, 60)
        self.assertTrue(result['counter_target_reached'])
        self.assertEqual(exchange.call_count, 12)
        self.assertEqual(board.send_verified_report.call_count, 11)

    def test_slice_bounds_reject_long_streams(self):
        for slices in (0, 61, 72):
            with self.assertRaises(ValueError):
                test_reports(1, 1, slices)

    def test_packets_are_bounded_and_finish_like_dll(self):
        packets = test_reports(-1, 1)
        self.assertEqual(packets[0][62:], bytes([6, 4]))
        self.assertEqual(packets[1][62:], bytes([1, 9]))
        self.assertEqual(packets[1][0], 0x20)
        self.assertEqual(packets[1], bytes([0x20]) + bytes(61) + bytes([1, 9]))
        self.assertEqual(sum(packets[0][i+1] for i in range(0, 60, 10)), 4)
        self.assertEqual(sum(packets[0][i+2] for i in range(0, 60, 10)), 4)
        self.assertEqual(packets[0][0], 2)

    def test_completes_only_when_both_axes_and_ready_match(self):
        board = MagicMock()
        board.read.return_value = bytes(63) + b'\x09'
        packets = test_reports(-1, 1)
        with patch('research.tools.coordinated_test.exchange_status', side_effect=[
                status(10, 20), status(6, 20), status(6, 24, False), status(6, 24)]), \
             patch('research.tools.coordinated_test.time.sleep'):
            result = run_test(board, b'poll', packets, None, -1, 1)
        self.assertTrue(result['counter_target_reached'])
        self.assertFalse(result['timing_and_path_verified'])
        self.assertEqual([c.args[0] for c in board.send_verified_report.call_args_list], packets)

    def test_poll_failure_attempts_reset(self):
        board = MagicMock()
        board.read.return_value = bytes(63) + b'\x09'
        packets = test_reports(1, 1)
        with patch('research.tools.coordinated_test.exchange_status', side_effect=[
                status(0, 0), RuntimeError('timeout')]):
            with self.assertRaises(RuntimeError):
                run_test(board, b'poll', packets, None, 1, 1)
        self.assertEqual(board.send_verified_report.call_args.args[0], bytes(63) + b'\x63')

    def test_missing_end_reply_attempts_reset_before_any_more_polls(self):
        board = MagicMock()
        board.read.return_value = b''
        with patch('research.tools.coordinated_test.exchange_status', return_value=status(0, 0)) as exchange:
            with self.assertRaisesRegex(RuntimeError, 'end-of-stream reply'):
                run_test(board, b'poll', test_reports(1, 1), None, 1, 1)
        self.assertEqual(exchange.call_count, 1)
        board.read.assert_called_once_with(500)
        self.assertEqual(board.send_verified_report.call_args.args[0], bytes(63) + b'\x63')

    def test_busy_preflight_never_sends_motion(self):
        board = MagicMock()
        with patch('research.tools.coordinated_test.exchange_status', return_value=status(0, 0, False)):
            with self.assertRaises(RuntimeError):
                run_test(board, b'poll', test_reports(1, 1), None, 1, 1)
        board.send_verified_report.assert_not_called()
