"""
Fake oven for testing the protocol code and controllers without hardware.

The thermal model is a made-up two-node lag model (heater rods -> chamber air),
NOT identified from a real RF-A250. It is only meant to exercise the code and
show qualitatively why a high-gain step-setpoint PID overshoots a laggy plant.
Run `python reflow.py step ...` on the real oven to get real numbers.
"""
from __future__ import annotations

from oven import crc16, frame


class VirtualClock:
    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


class FakeOvenSerial:
    def __init__(self, clock: VirtualClock, ambient: float = 25.0):
        self.clock = clock
        self.amb = ambient
        self.Th = ambient     # heater element temperature
        self.T = ambient      # chamber/sensor temperature
        self.heat = 0         # 0..100
        self.o35 = 0
        self.o36 = 0
        self._last = 0.0
        self._rx = b""
        self.linked = False

    # physics
    def _advance(self):
        dt = self.clock.now() - self._last
        self._last = self.clock.now()
        steps = max(1, int(dt / 0.05))
        h = dt / steps
        for _ in range(steps):
            circ = 1.0 + (0.6 if self.o36 else 0.0)            # circulation improves rod->air coupling
            cool = 0.004 + 0.02 * (self.o35 / 100.0)          # forced cooling
            dTh = (self.heat / 100.0) * 9.0 - (self.Th - self.T) * 0.045 * circ - (self.Th - self.amb) * 0.002
            dT = (self.Th - self.T) * 0.05 * circ - (self.T - self.amb) * cool
            self.Th += dTh * h
            self.T += dT * h

    # serial-like API
    def reset_input_buffer(self):
        self._rx = b""

    def flush(self):
        pass

    def close(self):
        pass

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def write(self, tx: bytes):
        self._advance()
        if tx == b"\x02":
            self.linked = True
            self._rx = b"\x06"
            return
        payload, c = tx[:-2], tx[-2:]
        if crc16(payload) != (c[0] << 8 | c[1]):
            return  # silently ignore bad frames
        if payload == b"\x03\x00\x00":
            self.linked = False
            self._rx = frame(b"\x06\x00\x00")[:3]
        elif payload == bytes([0xA1, 0x38, 0x02]):
            t = int(self.T)  # oven reports whole degrees
            self._rx = frame(bytes([0xA1, 0x38, 0x02, t >> 8, t & 0xFF]))
        elif payload[0] == 0xA0 and len(payload) == 4:
            cmd, val = payload[1], payload[3]
            if cmd == 0x34:
                self.heat = min(val, 100)
            elif cmd == 0x35:
                self.o35 = val
            elif cmd == 0x36:
                self.o36 = val
            self._rx = frame(payload[:3])
