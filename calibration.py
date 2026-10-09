"""
Oven-sensor -> board-temperature calibration.

A calibration file is JSON:  {"points": [[oven_C, board_C], ...]}
e.g. {"points": [[100, 92], [150, 147], [200, 203], [240, 247]]}

Between points the correction is linear. Above the last point the last segment
is extended. Below the first point the correction fades to zero at room
temperature (an implicit 25 C -> 25 C anchor), since a cold oven and a cold
board read the same. With one point it is an offset that fades in from 25 C.
"""
from __future__ import annotations

import json

AMBIENT = 25.0


class Calibration:
    def __init__(self, points=None):
        pts = sorted((float(a), float(b)) for a, b in (points or []))
        # drop duplicate oven readings (keep the last)
        dedup = {}
        for a, b in pts:
            dedup[a] = b
        self.points = sorted(dedup.items())
        self._anchor = AMBIENT if self.points and self.points[0][0] > AMBIENT + 5 else None

    def _pts(self):
        p = list(self.points)
        if self._anchor is not None:
            p.insert(0, (self._anchor, self._anchor))
        return p

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        return cls(data["points"] if isinstance(data, dict) else data)

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"points": [[round(a, 1), round(b, 1)] for a, b in self.points]}, f, indent=1)

    def __bool__(self):
        return bool(self.points)

    def board(self, oven: float) -> float:
        """Estimated board temperature for an oven-sensor reading."""
        p = self._pts()
        if not p:
            return float(oven)
        if len(p) == 1:
            return oven + (p[0][1] - p[0][0])
        if oven <= p[0][0]:
            if self._anchor is not None:
                return float(oven)
            (a0, b0), (a1, b1) = p[0], p[1]
        elif oven >= p[-1][0]:
            (a0, b0), (a1, b1) = p[-2], p[-1]
        else:
            for (a0, b0), (a1, b1) in zip(p, p[1:]):
                if a0 <= oven <= a1:
                    break
        return b0 + (b1 - b0) * (oven - a0) / (a1 - a0)

    def oven(self, board: float) -> float:
        """Inverse: oven reading that corresponds to a board temperature."""
        lo, hi = -50.0, 400.0
        for _ in range(60):
            mid = (lo + hi) / 2
            if self.board(mid) < board:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def describe(self):
        if not self.points:
            return "none"
        return ", ".join(f"{a:.0f}→{b:.0f}" for a, b in self.points)
