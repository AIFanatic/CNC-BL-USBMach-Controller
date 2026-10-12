"""Local CNC UI: python -m src.web_ui.server --profile machine.json [--allow-write]."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import queue
import threading
import uuid

from ..controller import (Controller,
                        decode_status, exchange_status, initialize_board, load_profile)
from ..gcode_runner import compile_program
from ..motion import execute_line, plan_linear, planner_limits
from ..planner import combined_plan


class Stopped(RuntimeError):
    pass


class CancelBoard:
    """Check cancellation at USB boundaries; always permit the core's reset."""
    def __init__(self, board, event, on_report=None):
        self.board, self.event = board, event
        self.on_report = on_report

    def check(self):
        if self.event.is_set():
            raise Stopped('Stop requested')

    def read(self, timeout_ms=100):
        self.check()
        result = self.board.read(min(timeout_ms, 50))
        if self.on_report and len(result) == 64 and result[63] in (0, 3, 5):
            self.on_report(decode_status(result))
        self.check()
        return result

    def send_verified_report(self, payload):
        if payload[63] != 0x63:
            self.check()
        return self.board.send_verified_report(payload)


class Controls:
    def __init__(self, profile_path, allow_write=False, factory=Controller):
        self.profile = json.loads(profile_path.read_text())
        self.setup, self.polling = load_profile(profile_path)
        limits = planner_limits(self.profile)
        self.allow_write, self.factory = allow_write, factory
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.jobs = queue.Queue()
        self.program = None
        self.state = dict(allow_write=allow_write, connected=False, busy=False,
                          message='Ready to preview', position_mm=None, progress=None,
                          program_id=None, tool_mm=None,
                          limits={axis: dict(
                              feed_mm_min=min(settings['feed_units_min'], 60 * limits['max_rate_counts_s'] / settings['steps_per_unit']),
                              acceleration_mm_s2=min(settings['acceleration_units_s2'], limits['max_acceleration_counts_s2'] / settings['steps_per_unit']),
                              line_distance_mm=limits['max_move_counts'] / settings['steps_per_unit'])
                              for axis, settings in zip('XYZ', self.profile['axes'])})
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def position(self, status):
        return {axis: status['axis_counters'][axis] / self.profile['axes'][i]['steps_per_unit']
                for i, axis in enumerate('XYZ')}

    def submit(self, action, data=None):
        with self.lock:
            if not self.allow_write:
                raise ValueError('Start the server with --allow-write to enable hardware controls')
            if self.state['busy']:
                raise ValueError('A command is already running')
            if action != 'connect' and not self.state['connected']:
                raise ValueError('Connect the board first')
            self.cancel.clear()
            self.state.update(busy=True, message=action.capitalize(), progress=None)
            if action in ('connect', 'jog', 'run'):
                self.state.update(program_id=data[0] if action == 'run' else None, tool_mm=None)
            self.jobs.put((action, data))
        return {'accepted': True}

    def preview(self, text):
        moves = compile_program(text, self.profile)
        if not moves:
            raise ValueError('File contains no moves')
        token = uuid.uuid4().hex
        with self.lock:
            self.program = (token, moves)
        return dict(id=token, moves=[move.summary() for move in moves],
                    seconds=sum(move.plan.duration_seconds for move in moves))

    def request(self, action, data):
        if action == 'preview':
            return self.preview(data['text'])
        if action == 'stop':
            with self.lock:
                if self.state['busy']:
                    self.cancel.set()
                    self.state['message'] = 'Stop requested; waiting for board'
                    return {'accepted': True}
            return self.submit('stop')
        if action == 'jog':
            axis = data['axis']
            if axis not in ('X', 'Y', 'Z'):
                raise ValueError('Axis must be X, Y or Z')
            delta = [0, 0, 0]
            delta['XYZ'.index(axis)] = data['distance']
            return self.submit('jog', plan_linear(self.profile, *delta, feed=data['feed']))
        if action == 'run':
            with self.lock:
                if self.program is None or data['id'] != self.program[0]:
                    raise ValueError('Preview this file before running')
                moves = self.program[1]
            return self.submit('run', (data['id'], moves))
        if action in ('connect', 'status'):
            return self.submit(action)
        raise ValueError('Unknown action')

    def work(self):
        board = None
        while True:
            action, data = self.jobs.get()
            if action == 'close':
                if board:
                    board.close()
                return
            try:
                if action == 'connect':
                    if board is None:
                        board = self.factory()
                    result = initialize_board(board, self.setup, self.polling)
                    self.update(connected=True, position_mm=self.position(result['status']))
                elif action in ('status', 'stop'):
                    if action == 'stop':
                        board.send_verified_report(bytes(63) + b'\x63')
                    status = exchange_status(board, self.polling)
                    self.update(position_mm=self.position(status))
                    if action == 'stop' and not status['dll_ready_bit']:
                        raise RuntimeError('Reset sent, but board is not ready')
                else:
                    moves = [] if action == 'jog' else data[1]
                    line_indices = {m.source_line: i for i, m in enumerate(moves, 1)}
                    origin = None
                    def observe(status):
                        nonlocal origin
                        values = dict(position_mm=self.position(status))
                        if action == 'run':
                            counts = status['axis_counters']
                            if origin is None:
                                origin = dict(counts)
                            values['tool_mm'] = {
                                axis: (((counts[axis] - origin[axis] + 2**31) % 2**32) - 2**31)
                                / self.profile['axes'][i]['steps_per_unit']
                                for i, axis in enumerate('XYZ')}
                            if status.get('raw'):
                                source_line = int.from_bytes(bytes.fromhex(status['raw'])[28:32], 'little')
                                if source_line in line_indices:
                                    values['progress'] = dict(move=line_indices[source_line], total=len(moves), source_line=source_line)
                        self.update(**values)
                    wrapped = CancelBoard(board, self.cancel, observe)
                    self.update(progress=dict(move=1, total=len(moves) or 1,
                                              source_line=moves[0].source_line if moves else None))
                    result = execute_line(wrapped, combined_plan(moves) if moves else data, self.polling)
                    observe(result['status'])
                    if moves:
                        self.update(progress=dict(move=len(moves), total=len(moves), source_line=moves[-1].source_line))
                if self.cancel.is_set():
                    raise Stopped('Stop requested')
                self.update(message='Completed by reported counters' if action in ('jog', 'run') else 'Board ready')
            except Exception as exc:
                self.update(message=str(exc))
                if board and self.cancel.is_set():
                    try:
                        # Also covers cancellation between lines or during preflight.
                        board.send_verified_report(bytes(63) + b'\x63')
                        status = exchange_status(board, self.polling)
                        if action == 'run':
                            observe(status)
                        self.update(position_mm=self.position(status), message=(
                            'Stopped; board ready' if status['dll_ready_bit'] else 'Reset sent; board not ready'))
                    except Exception as stop_error:
                        self.update(message=f'Stop could not be confirmed: {stop_error}')
                elif board:
                    board.close()
                    board = None
                    self.update(connected=False)
            finally:
                self.update(busy=False)
                self.jobs.task_done()

    def close(self):
        self.cancel.set()
        self.jobs.put(('close', None))
        self.worker.join(timeout=4)


STATIC = Path(__file__).parent / 'static'


def handler_for(controls):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, value):
            body = json.dumps(value).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def local_request(self):
            port = self.server.server_port
            hosts = (f'127.0.0.1:{port}', f'localhost:{port}')
            host = self.headers.get('Host')
            origin = self.headers.get('Origin')
            return host in hosts and (origin is None or origin == f'http://{host}')

        def do_GET(self):
            if not self.local_request():
                return self.reply(403, {'error': 'Local requests only'})
            if self.path == '/api/state':
                return self.reply(200, controls.snapshot())
            files = {'/': ('index.html', 'text/html'), '/app.js': ('app.js', 'text/javascript'),
                     '/style.css': ('style.css', 'text/css')}
            if self.path not in files:
                return self.reply(404, {'error': 'Not found'})
            name, mime = files[self.path]
            body = (STATIC / name).read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if not self.local_request() or self.headers.get('X-CNC-UI') != '1':
                return self.reply(403, {'error': 'Local UI requests only'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 1024 * 1024:
                    raise ValueError('Request must be between 1 byte and 1 MB')
                data = json.loads(self.rfile.read(size))
                if not self.path.startswith('/api/'):
                    raise ValueError('Unknown endpoint')
                result = controls.request(self.path[5:], data)
                self.reply(200, result)
            except (ValueError, KeyError, TypeError, RuntimeError) as exc:
                self.reply(400, {'error': str(exc)})

        def log_message(self, *_):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=Path('machine.json'))
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--allow-write', action='store_true')
    args = parser.parse_args()
    controls = Controls(args.profile, args.allow_write)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(controls))
    print(f'Open http://127.0.0.1:{server.server_port} ({"hardware enabled" if args.allow_write else "preview only"})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        controls.close()
        server.server_close()


if __name__ == '__main__':
    main()
