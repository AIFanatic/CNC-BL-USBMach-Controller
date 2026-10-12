"""MIT-licensed continuous planning, using v² = u² + 2as.

Tangent joins carry speed. Corners and G0/G1 changes stop exactly; no corner
rounding is performed. XY arcs are sampled directly with curvature speed limits.
Timelines are sampled at 4 ms across tangent joins,
with stop points aligned to the sample grid. No third-party planner code.
"""
import math
from dataclasses import dataclass

from .controller import MAX_MOVE_COUNTS
from .motion import LinePlan, linear_limits, planner_limits
from .trajectory import SLICE_SECONDS, motion_record


@dataclass
class Arc:
    center: tuple
    radius: float
    angle: float
    sweep: float
    z: float

    def point(self, fraction):
        angle = self.angle + self.sweep * fraction
        return (self.center[0] + self.radius * math.cos(angle),
                self.center[1] + self.radius * math.sin(angle), self.z)

    def tangent(self, fraction):
        angle = self.angle + self.sweep * fraction
        sign = 1 if self.sweep > 0 else -1
        return (-sign * math.sin(angle), sign * math.cos(angle), 0.)

    def bounds(self):
        angles = [self.angle, self.angle + self.sweep]
        sign = 1 if self.sweep > 0 else -1
        angles += [k * math.pi / 2 for k in range(4)
                   if ((k * math.pi / 2 - self.angle) * sign) % math.tau <= abs(self.sweep) + 1e-12]
        points = [(self.center[0] + self.radius * math.cos(a),
                   self.center[1] + self.radius * math.sin(a)) for a in angles]
        return [(min(p[i] for p in points), max(p[i] for p in points)) for i in range(2)]


def make_arc(start, end, words, clockwise, scales, max_counts=MAX_MOVE_COUNTS):
    """XY center offsets are incremental; positive R selects the minor arc."""
    if start[2] != end[2]:
        raise ValueError('Helical arcs are unsupported; keep Z unchanged')
    chord = math.dist(start[:2], end[:2])
    if 'R' in words:
        if 'I' in words or 'J' in words or not chord or words['R'] == 0:
            raise ValueError('R arcs require distinct XY endpoints and no I/J')
        radius = abs(words['R'])
        if chord > 2 * radius:
            raise ValueError('Arc radius is smaller than half the chord')
        height = radius * math.sqrt(max(0., 1 - (chord / radius / 2)**2))
        middle = tuple((a+b)/2 for a, b in zip(start, end))
        normal = (-(end[1]-start[1])/chord, (end[0]-start[0])/chord)
        centers = [tuple(middle[i]+sign*height*normal[i] for i in range(2)) for sign in (1, -1)]
    elif 'I' in words or 'J' in words:
        center = (start[0]+words.get('I', 0.), start[1]+words.get('J', 0.))
        radius, end_radius = math.dist(start[:2], center), math.dist(end[:2], center)
        if not radius or abs(radius-end_radius) > 2 * max(1/scales[0], 1/scales[1]):
            raise ValueError('Arc start/end radii disagree, or radius is zero')
        if chord:
            # Fit the center to quantized endpoints within the rounding tolerance.
            shift = (end_radius-radius) * (end_radius+radius) / (2*chord)
            if abs(shift) > 2 * max(1/scales[0], 1/scales[1]):
                raise ValueError('Arc center fit exceeds rounding tolerance')
            center = tuple(center[i]+shift*(end[i]-start[i])/chord for i in range(2))
        centers = [center]
    else:
        raise ValueError('G2/G3 require I/J offsets or R')
    for center in centers:
        radius = math.dist(start[:2], center)
        angle = math.atan2(start[1]-center[1], start[0]-center[0])
        finish = math.atan2(end[1]-center[1], end[0]-center[0])
        sweep = -((angle-finish) % math.tau) if clockwise else (finish-angle) % math.tau
        if not chord:
            sweep = -math.tau if clockwise else math.tau
        if not all(math.isfinite(v) for v in (*center, radius, sweep)) or radius <= 0 or sweep == 0:
            raise ValueError('Arc geometry cannot be represented')
        if 'R' not in words or (abs(sweep) <= math.pi+1e-12 if words['R'] > 0 else abs(sweep) >= math.pi-1e-12):
            arc = Arc(center, radius, angle, sweep, start[2])
            if any(abs((p-start[i])*scales[i]) > max_counts+1e-7
                   for i, bounds in enumerate(arc.bounds()) for p in bounds):
                raise ValueError(f'Arc excursion exceeds planner_limits.max_move_counts ({max_counts:g}) on an axis')
            return arc
    raise ValueError('Cannot construct R arc')


def curve_limits(profile, arc, feed):
    # Reserve half the acceleration for tangential motion and half for v²/r.
    limits = planner_limits(profile)
    axes = profile['axes'][:2]
    budget = min(min(a['acceleration_units_s2'], limits['max_acceleration_counts_s2']/a['steps_per_unit']) for a in axes) / 2
    speed = min(feed/60, math.sqrt(budget*arc.radius),
                *(min(a['feed_units_min']/60, limits['max_rate_counts_s']/a['steps_per_unit']) for a in axes))
    if not math.isfinite(speed) or speed <= 0 or not math.isfinite(budget) or budget <= 0:
        raise ValueError('Arc speed or acceleration cannot be represented')
    return speed, budget


def combined_plan(moves):
    """Reuse the existing stream executor for an entire preplanned program."""
    records = [record for move in moves for record in move.plan.records]
    return LinePlan(tuple(sum(m.plan.counts[i] for m in moves) for i in range(3)),
                    tuple(sum(m.plan.distance_mm[i] for m in moves) for i in range(3)),
                    max(m.plan.feed_mm_min for m in moves),
                    max(m.plan.acceleration_mm_s2 for m in moves),
                    len(records) * SLICE_SECONDS, records)


def plan_program(moves, profile):
    """Apply braking lookahead, then sample one timeline per exact-stop group."""
    limits = [(m.plan.counts, m.plan.distance_mm, *curve_limits(profile, m.arc, m.requested_feed))
              if m.arc else linear_limits(profile, *m.plan.distance_mm, feed=m.requested_feed) for m in moves]
    lengths = [m.arc.radius * abs(m.arc.sweep) if m.arc else math.hypot(*limit[1])
               for m, limit in zip(moves, limits)]
    def tangent(i, fraction):
        return moves[i].arc.tangent(fraction) if moves[i].arc else tuple(d / lengths[i] for d in limits[i][1])
    # Squared boundary speeds. A finite acceleration cannot traverse an exact
    # direction discontinuity at nonzero velocity, so only straight joins blend.
    speeds = [0.] * (len(moves) + 1)
    for i in range(1, len(moves)):
        if ((moves[i-1].mode == 'G0') == (moves[i].mode == 'G0') and
                math.dist(tangent(i-1, 1), tangent(i, 0)) < 1e-10):
            speeds[i] = min(limits[i-1][2], limits[i][2]) ** 2
    for i in reversed(range(len(moves))):
        speeds[i] = min(speeds[i], speeds[i+1] + 2 * limits[i][3] * lengths[i])
    for i in range(len(moves)):
        speeds[i+1] = min(speeds[i+1], speeds[i] + 2 * limits[i][3] * lengths[i])
    phases = []
    for i, move in enumerate(moves):
        counts, distances, cap, acceleration = limits[i]
        start, end = math.sqrt(speeds[i]), math.sqrt(speeds[i+1])
        peak = min(cap, math.sqrt(acceleration * lengths[i] + (speeds[i] + speeds[i+1]) / 2))
        up, down = max(0., (peak-start) / acceleration), max(0., (peak-end) / acceleration)
        up_distance = (start+peak) * up / 2
        cruise = max(0., (lengths[i] - up_distance - (end+peak)*down/2) / peak)
        duration = up + cruise + down
        if not math.isfinite(duration) or duration > planner_limits(profile)['max_line_seconds']:
            raise ValueError(f"Line {move.source_line}: exceeds planner_limits.max_line_seconds ({planner_limits(profile)['max_line_seconds']:g} s)")
        phases.append((start, peak, end, up, cruise, duration, up_distance))
        move.plan = LinePlan(counts, distances, peak*60, acceleration, 0., [])
    scales = [a['steps_per_unit'] for a in profile['axes'][:3]]
    old_counts, first = (0, 0, 0), 0
    for last in range(len(moves)):
        if speeds[last+1] != 0:
            continue
        # Slow a stop-to-stop group slightly to fit whole 4 ms intervals. This
        # preserves the exact stop vertex and avoids rounding every line's time.
        duration = sum(p[5] for p in phases[first:last+1])
        samples = max(1, math.ceil(duration / SLICE_SECONDS))
        index, elapsed = first, 0.
        for sample in range(1, samples+1):
            t = sample * duration / samples
            while index < last and t > elapsed + phases[index][5]:
                elapsed += phases[index][5]
                index += 1
            start, peak, end, up, cruise, line_time, up_distance = phases[index]
            local = min(line_time, max(0., t-elapsed))
            acceleration = limits[index][3]
            if local < up:
                distance = start*local + acceleration*local*local/2
            elif local < up + cruise:
                distance = up_distance + peak*(local-up)
            else:
                left = max(0., line_time-local)
                distance = lengths[index] - end*left - acceleration*left*left/2
            fraction = min(1., max(0., distance / lengths[index]))
            origin = (0., 0., 0.) if index == 0 else moves[index-1].target_mm
            point = moves[index].arc.point(fraction) if moves[index].arc else tuple(o+d*fraction for o, d in zip(origin, limits[index][1]))
            position = tuple(round(p*scale) for p, scale in zip(point, scales))
            if sample == samples:
                position = tuple(round(p*s) for p, s in zip(moves[last].target_mm, scales))
            delta = [new-old for new, old in zip(position, old_counts)]
            moves[index].plan.records.append(motion_record(delta+[0, 0], moves[index].source_line))
            old_counts = position
        first = last+1
    for move in moves:
        move.plan.duration_seconds = len(move.plan.records) * SLICE_SECONDS
    return combined_plan(moves)
