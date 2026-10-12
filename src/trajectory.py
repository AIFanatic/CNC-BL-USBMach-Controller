"""BL-UsbMach-V22.dll firmware-21 trajectory packet packing.

The 24-bit field is a Mach3 line ID. Motion slices are 4 ms.
Hardware tests verified coordinated counters, batching and stream completion.
Integer axis values only; the DLL also accepts fractional doubles.
"""


SLICE_SECONDS = 0.004


def motion_record(axes, line_id):
    """Pack five signed axis values and the Mach3 line ID."""
    if len(axes) != 5 or any(type(value) is not int or abs(value) > 400 for value in axes):
        raise ValueError('Provide five integer axis values, each within -400..400')
    if type(line_id) is not int or not 0 <= line_id < 0xffffff:
        raise ValueError('line_id must be an integer in 0..0xfffffe; 0xffffff is reserved')
    magnitudes = [abs(value) for value in axes]
    signs = sum(1 << i for i, value in enumerate(axes) if value > 0)
    # X high bits share the direction byte; Y/A/Z/B high bits share byte 9.
    first = signs | ((magnitudes[0] >> 8) << 6)
    high = sum((magnitudes[i] >> 8) << shift for i, shift in ((1, 0), (2, 2), (3, 4), (4, 6)))
    return bytes([first, *(value & 255 for value in magnitudes)]) + line_id.to_bytes(3, 'little') + bytes([high])


def motion_report(records, command=4):
    """Pack 1..6 records; command 9 marks end of the stream as in 0x35d0."""
    if not 1 <= len(records) <= 6 or any(len(record) != 10 for record in records):
        raise ValueError('Provide 1..6 ten-byte records')
    if command not in (4, 5, 9):
        raise ValueError('Supported reconstructed commands: 4, 5, 9')
    payload = bytearray(64)
    payload[:10 * len(records)] = b''.join(records)
    if command == 9:
        payload[10 * (len(records) - 1)] |= 0x20
    payload[62] = len(records)
    payload[63] = command
    return bytes(payload)


def stream_progress(report):
    """Fields consumed by DLL reset/poll paths; queue units remain candidate."""
    if len(report) != 64 or report[63] != 3:
        raise ValueError('Expected a 64-byte command-3 status reply')
    return {'line_id': int.from_bytes(report[28:32], 'little'),
            'batch_threshold': int.from_bytes(report[32:36], 'little')}
