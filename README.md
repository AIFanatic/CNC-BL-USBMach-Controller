# BL-USBMach Controller

A Python CNC controller for Mach3-compatible BL-USBMach 2.0 boards that
communicates directly over USB using the board’s existing firmware—without
flashing GRBL, replacing the board, or running Mach3 and its Windows DLL.

Built through reverse engineering of the original protocol, it provides XYZ
jogging, coordinated G-code motion, XY arcs, position reporting and a local web
interface with toolpath previews. Tested on a board reporting firmware code **21**.

## Setup

Run commands from the project root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

## Machine settings

Edit `machine.json` before starting the controller. Restart the web server after
changing it. The `axes` entries are ordered **X, Y, Z, A, B**. Each entry sets:

| Setting | Meaning |
|---|---|
| `steps_per_unit` | Motor step counts per millimetre |
| `feed_units_min` | Maximum axis feed in mm/min |
| `acceleration_units_s2` | Maximum axis acceleration in mm/s² |

The included XYZ settings are 818 steps/mm, 60 mm/min and 10 mm/s². Calibrate
these for your machine. G-code `F` requests a path feed; the planner caps it to
the axis settings and the count-rate limits below. Leave the USB configuration
reports and pin mapping unchanged unless adapting the board wiring.

## Web interface

```bash
sudo .venv/bin/python -m src.web_ui.server --profile machine.json --allow-write
```

Open **http://127.0.0.1:8000**, click **Connect / initialize**, then jog or open
G-code and select **Plan & preview → Run program**. Omit `--allow-write` to use
preview mode only.

The interface shows the XY toolpath and reported tool position. The feed box
controls jogging; program feed comes from `F` in G-code. **Stop motion** attempts
to cancel the current move. Closing the browser tab does not stop a program.

## Command line

```bash
# Identify the board
.venv/bin/python -m src.controller identify

# Interactive jog, status and local zero controls
.venv/bin/python -m src.controller console --profile machine.json --allow-write

# Preview your G-code file
.venv/bin/python -m src.gcode_runner program.ngc --profile machine.json

# Run it
sudo .venv/bin/python -m src.gcode_runner program.ngc --profile machine.json --allow-write
```

The console accepts `status`, `zero`, `jog X 1`, `jog Y -1`, `stop` and `quit`.
Ctrl-C attempts to stop active motion. Use one controller process at a time.

## G-code support

- XYZ linear motion: `G0`, `G1`.
- XY arcs: `G2`, `G3`, `G17`; incremental I/J centers or R radius.
- Millimetres and coordinate modes: `G21`, `G90`, `G91`, `G91.1`, `G94`.
- Feed `F`, line numbers `N`, comments, `%`, and standalone `M2`/`M30`.

Programs are validated before motion starts. Straight and tangent joins retain
speed; sharp corners and reversals stop. I/J supports full circles; positive R
selects the smaller arc and negative R the larger arc.

**Program zero is the position when Run starts**, independent of console zero.
Reported positions come from board counters, not encoders.

Not yet supported: spindle/coolant commands, tool changes, rotary axes, homing,
work offsets, XZ/YZ or helical arcs, inches, pause/resume and feed override.

## Planner limits

Set these in the `planner_limits` object in `machine.json`:

```json
"planner_limits": {
  "max_rate_counts_s": 1000,
  "max_acceleration_counts_s2": 10000,
  "max_move_counts": 10000,
  "max_line_seconds": 30
}
```

| Setting | Controls |
|---|---|
| `max_rate_counts_s` | Maximum step counts per second on each axis |
| `max_acceleration_counts_s2` | Maximum count acceleration on each axis |
| `max_move_counts` | Maximum excursion from a line/arc's start on each axis |
| `max_line_seconds` | Maximum planned duration of one line or arc |

These apply to G-code, coordinated line commands and web jogging. Values must
be positive. At 818 steps/mm, 10,000 counts means approximately **12.22 mm**,
and 1,000 counts/s means **73.35 mm/min**. The axis profile can cap speed lower.

For example, set `max_line_seconds` to **60** to permit a single 10 mm diameter
circle at 60 mm/min. Increasing this allows a longer move; it does not make it
faster. Increase `max_move_counts` to allow longer lines or larger arc excursions.
The limits are software planning bounds, not machine end-of-travel limits.

Coded with `GPT-6.1-Sol low` so expect the usual.