#!/usr/bin/env python3
"""Uniform-arclength resampling of a polyline path (numpy only)."""
import numpy as np


def resample_path(xs, ys, ds):
    """
    xs, ys : 1-D arrays of the raw plan vertices (map frame).
    ds     : target spacing in metres.
    Returns (s, rx, ry, ryaw):
      s    : cumulative arc length at each resampled point  (len N)
      rx,ry: resampled coordinates                          (len N)
      ryaw : heading at each point (atan2 of forward diff)  (len N)
    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if xs.size < 2:
        return np.array([0.0]), xs, ys, np.zeros_like(xs)

    seg = np.hypot(np.diff(xs), np.diff(ys))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total < 1e-6:
        return np.array([0.0]), xs[:1], ys[:1], np.zeros(1)

    n = max(2, int(np.ceil(total / ds)) + 1)
    s = np.linspace(0.0, total, n)
    rx = np.interp(s, cum, xs)
    ry = np.interp(s, cum, ys)

    ryaw = np.zeros(n)
    ryaw[:-1] = np.arctan2(np.diff(ry), np.diff(rx))
    ryaw[-1] = ryaw[-2] if n >= 2 else 0.0
    return s, rx, ry, ryaw