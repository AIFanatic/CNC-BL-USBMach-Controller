import threading
import struct
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from src.web_ui.server import CancelBoard, Controls, Stopped


STATUS = {'dll_ready_bit': True, 'axis_counters': dict.fromkeys('XYZAB', 0)}


class WebTests(unittest.TestCase):
    def test_preview_does_not_open_hardware(self):
        factory = Mock()
        controls = Controls(Path('machine.json'), factory=factory)
        try:
            result = controls.preview('G21 G90\nG1 X1 F60\nM2')
            self.assertEqual(len(result['moves']), 1)
            factory.assert_not_called()
            with self.assertRaisesRegex(ValueError, 'allow-write'):
                controls.request('connect', {})
            with self.assertRaisesRegex(ValueError, 'Unsupported'):
                controls.preview('G18 G2 X1 F60')
        finally:
            controls.close()

    def test_cancel_blocks_motion_but_allows_reset(self):
        board, event = Mock(), threading.Event()
        wrapped = CancelBoard(board, event)
        event.set()
        with self.assertRaises(Stopped):
            wrapped.send_verified_report(bytes(63) + b'\x04')
        with self.assertRaises(Stopped):
            wrapped.read()
        reset = bytes(63) + b'\x63'
        wrapped.send_verified_report(reset)
        board.send_verified_report.assert_called_once_with(reset)
        board.read.assert_not_called()

    def test_worker_serializes_motion_and_confirms_stop(self):
        board = Mock()
        started, release = threading.Event(), threading.Event()
        def moving(wrapped, plan, polling):
            started.set()
            release.wait(2)
            wrapped.check()
            return {'status': STATUS}
        with patch('src.web_ui.server.initialize_board', return_value={'status': STATUS}), \
             patch('src.web_ui.server.exchange_status', return_value=STATUS), \
             patch('src.web_ui.server.execute_line', side_effect=moving):
            controls = Controls(Path('machine.json'), True, factory=lambda: board)
            try:
                controls.request('connect', {})
                controls.jobs.join()
                controls.request('jog', {'axis': 'X', 'distance': 1, 'feed': 60})
                self.assertTrue(started.wait(1))
                with self.assertRaisesRegex(ValueError, 'already running'):
                    controls.request('status', {})
                controls.request('stop', {})
                release.set()
                controls.jobs.join()
                self.assertEqual(controls.snapshot()['message'], 'Stopped; board ready')
                self.assertFalse(controls.snapshot()['busy'])
                board.send_verified_report.assert_called_with(bytes(63) + b'\x63')
            finally:
                release.set()
                controls.close()

    def test_run_requires_matching_preview(self):
        controls = Controls(Path('machine.json'))
        try:
            with self.assertRaisesRegex(ValueError, 'Preview'):
                controls.request('run', {'id': 'stale'})
        finally:
            controls.close()

    def test_stream_reports_update_tool_relative_to_program_start(self):
        board = Mock()
        def report(x, command, source_line=0):
            payload = bytearray(64)
            struct.pack_into('<i', payload, 4, x)
            struct.pack_into('<I', payload, 28, source_line)
            payload[63] = command
            return bytes(payload)
        board.read.side_effect = [report(1000, 3), report(1409, 5, 2), report(1818, 0, 2)]
        final = {**STATUS, 'axis_counters': {**STATUS['axis_counters'], 'X': 1818}}
        def moving(wrapped, plan, polling):
            wrapped.read()
            wrapped.read()
            # Position is visible before execute_line returns.
            self.assertEqual(controls.snapshot()['tool_mm']['X'], .5)
            self.assertAlmostEqual(controls.snapshot()['position_mm']['X'], 1409 / 818)
            self.assertEqual(controls.snapshot()['progress']['move'], 2)
            wrapped.read()
            return {'status': final}
        with patch('src.web_ui.server.initialize_board', return_value={'status': STATUS}), \
             patch('src.web_ui.server.execute_line', side_effect=moving):
            controls = Controls(Path('machine.json'), True, factory=lambda: board)
            try:
                controls.request('connect', {})
                controls.jobs.join()
                preview = controls.preview('G1 X.5 F600\nX1')
                controls.request('run', {'id': preview['id']})
                controls.jobs.join()
                self.assertEqual(controls.snapshot()['program_id'], preview['id'])
                self.assertEqual(controls.snapshot()['tool_mm']['X'], 1)
                self.assertEqual(controls.snapshot()['message'], 'Completed by reported counters')
                self.assertEqual(preview['moves'][0]['feed_mm_min'], 60)
                self.assertEqual(controls.snapshot()['limits']['X']['feed_mm_min'], 60)
            finally:
                controls.close()

    def test_identity_reports_do_not_update_motion_position(self):
        board, callback = Mock(), Mock()
        board.read.return_value = bytes(63) + b'\x62'
        CancelBoard(board, threading.Event(), callback).read()
        callback.assert_not_called()
