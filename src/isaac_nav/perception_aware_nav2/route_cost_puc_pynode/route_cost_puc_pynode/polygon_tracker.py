#!/usr/bin/env python3
"""
PolygonTracker — match-and-replace fusion of TrackedPolygon centroids with a
short-window history per track. Used by route_cost_puc_pynode for both:

  - Σρ_o (current snapshot of polygons in the visibility tube)
  - ρ_s  (centroid finite-diff velocity for dynamic labels)

Frames: assumes polygons are already in `map` (Lf3_Lf4 §2.5 confirmed).
"""
import math


def _centroid(points):
    """Centroid of geometry_msgs/Polygon.points. Drops a trailing duplicate
    point if the ring is explicitly closed (first == last)."""
    n = len(points)
    if n == 0:
        return None
    # Drop closing duplicate (the L2-confirmed live message has 5 pts with p0==p4).
    if n >= 2 and abs(points[0].x - points[-1].x) < 1e-9 and abs(points[0].y - points[-1].y) < 1e-9:
        pts = points[:-1]
    else:
        pts = points
    n = len(pts)
    sx = sum(p.x for p in pts)
    sy = sum(p.y for p in pts)
    return sx / n, sy / n


class PolygonTracker:
    """
    tracks = list of dicts:
      { 'label': int, 'cx': float, 'cy': float,
        'confidence': float, 'contributor_ratios': list[float],
        'history': list[(t_sec, cx, cy)],       # last few centroids w/ stamps
        'last_t': float }
    """
    def __init__(self, match_radius_m=0.4, decay_time_s=20.0,
                 history_window_s=2.0, max_history=8):
        self.match_r2 = match_radius_m * match_radius_m
        self.decay = decay_time_s
        self.hwin = history_window_s
        self.max_hist = max_history
        self.tracks = []

    def update(self, msg, now_sec):
        """Ingest a TrackedPolygon message at wall time `now_sec`."""
        c = _centroid(msg.polygon.points)
        if c is None:
            return
        cx, cy = c

        # Find nearest existing track within match_radius.
        nearest, best = None, self.match_r2
        for tr in self.tracks:
            dx, dy = tr['cx'] - cx, tr['cy'] - cy
            d2 = dx * dx + dy * dy
            if d2 < best:
                best = d2
                nearest = tr

        if nearest is None:
            nearest = {
                'label': int(msg.label),
                'cx': cx, 'cy': cy,
                'confidence': float(msg.confidence),
                'contributor_ratios': list(msg.contributor_ratios),
                'history': [],
                'last_t': now_sec,
            }
            self.tracks.append(nearest)
        else:
            nearest['label'] = int(msg.label)
            nearest['cx'], nearest['cy'] = cx, cy
            nearest['confidence'] = float(msg.confidence)
            nearest['contributor_ratios'] = list(msg.contributor_ratios)
            nearest['last_t'] = now_sec

        # Append history, drop entries older than hwin or beyond max_hist.
        nearest['history'].append((now_sec, cx, cy))
        nearest['history'] = [
            h for h in nearest['history'] if now_sec - h[0] <= self.hwin
        ][-self.max_hist:]

    def prune(self, now_sec):
        """Drop tracks whose last update is older than decay_time_s."""
        self.tracks = [
            tr for tr in self.tracks if now_sec - tr['last_t'] <= self.decay
        ]

    def active(self):
        return self.tracks

    @staticmethod
    def risk(track, a_uncertainty=1.0, b_fragility=0.7):
        """L2-equivalent normalized risk score ∈ [0,1]."""
        unc = max(0.0, min(1.0, 1.0 - track['confidence']))
        ratios = track['contributor_ratios']
        # Defensive: empty → max fragility (matches L2 layer behavior).
        frag = max(ratios) if ratios else 1.0
        frag = max(0.0, min(1.0, frag))
        denom = max(1e-6, a_uncertainty + b_fragility)
        return max(0.0, min(1.0, (a_uncertainty * unc + b_fragility * frag) / denom))

    @staticmethod
    def velocity(track, now_sec, min_dt=0.1):
        """Finite-diff (vx, vy) from history. Returns (0,0) if not enough."""
        h = track['history']
        if len(h) < 2:
            return 0.0, 0.0
        t0, x0, y0 = h[0]
        tN, xN, yN = h[-1]
        dt = tN - t0
        if dt < min_dt:
            return 0.0, 0.0
        return (xN - x0) / dt, (yN - y0) / dt