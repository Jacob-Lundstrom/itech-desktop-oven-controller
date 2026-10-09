"""
Serial driver for Huaqi Zhengbang "HLH" benchtop reflow ovens
(iTECH RF-A250 / RF-A350 / RF-A500, ZB2520HL / ZB3530HL / ZB5040HL).

Protocol recovered by static analysis of the vendor app ZBHLH_V1.0 Main.exe.
See README.md for the frame-by-frame description.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

try:
    import serial  # pyserial
except ImportError:  # allow the simulator to be used without pyserial
    serial = None

# Command bytes (2nd byte of an A0 "write" frame)
OUT_HEATER = 0x34   # heater power, 0..100 %
COOL_FAN   = 0x35   # cooling fan speed 0..100 %       (confirmed on RF-A250)
CONVECTION = 0x36   # convection fan enable 0/1        (confirmed on RF-A250)
OUT_35, OUT_36 = COOL_FAN, CONVECTION   # register-number aliases


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(payload: bytes) -> bytes:
    """Append CRC, HIGH byte first (note: opposite order to real Modbus RTU)."""
    c = crc16(payload)
    return payload + bytes([c >> 8, c & 0xFF])


class OvenError(Exception):
    pass


class Oven:
    def __init__(self, port: str | object, timeout: float = 1.0,
                 log: Optional[Callable[[str], None]] = None):
        """`port` is a COM port name ("COM3", "/dev/ttyUSB0") or an already-open
        serial-like object (used by the simulator)."""
        if isinstance(port, str):
            if serial is None:
                raise RuntimeError("pip install pyserial")
            self.ser = serial.Serial(port, 9600, bytesize=8, parity="N",
                                     stopbits=1, timeout=timeout, write_timeout=1)
        else:
            self.ser = port
        self.log = log
        self.linked = False

    # ---- low level -------------------------------------------------------
    def _xfer(self, tx: bytes, rxlen: int, settle: float = 0.0) -> bytes:
        self.ser.reset_input_buffer()           # vendor app purges before every write
        self.ser.write(tx)
        self.ser.flush()
        if settle:
            time.sleep(settle)
        rx = self.ser.read(rxlen)
        if self.log:
            self.log(f"TX {tx.hex(' ').upper():<20} RX {rx.hex(' ').upper()}")
        if len(rx) != rxlen:
            raise OvenError(f"No ack: sent {tx.hex(' ')}, got {len(rx)}/{rxlen} bytes ({rx.hex(' ')})")
        return rx

    def raw(self, payload_hex: str, rxlen: int) -> bytes:
        """Send arbitrary payload (CRC appended) - for exploration only."""
        return self._xfer(frame(bytes.fromhex(payload_hex)), rxlen)

    # ---- session ---------------------------------------------------------
    def link(self) -> None:
        """Enter PC-control mode: send 0x02, wait 500 ms, expect 0x06 (ACK)."""
        rx = self._xfer(b"\x02", 1, settle=0.5)
        if rx[0] != 0x06:
            raise OvenError(f"Link refused: {rx.hex()}")
        self.linked = True

    def unlink(self) -> None:
        """Leave PC-control mode: 03 00 00 + CRC, expect 3 bytes starting 0x06."""
        rx = self._xfer(frame(b"\x03\x00\x00"), 3)
        if rx[0] != 0x06:
            raise OvenError(f"Unlink refused: {rx.hex()}")
        self.linked = False

    # ---- commands --------------------------------------------------------
    def read_temp(self) -> int:
        """A1 38 02 -> 7-byte reply, temperature (integer C) big-endian in bytes 3..4."""
        rx = self._xfer(frame(bytes([0xA1, 0x38, 0x02])), 7)
        if crc16(rx[:5]) != (rx[5] << 8 | rx[6]):
            raise OvenError(f"Data check error: {rx.hex(' ')}")
        return rx[3] << 8 | rx[4]

    def set_output(self, cmd: int, value: int) -> None:
        """A0 <cmd> 01 <value> -> 5-byte reply (CRC over first 3 bytes)."""
        value = int(max(0, min(255, value)))
        rx = self._xfer(frame(bytes([0xA0, cmd, 0x01, value])), 5)
        if crc16(rx[:3]) != (rx[3] << 8 | rx[4]):
            raise OvenError(f"Data check error: {rx.hex(' ')}")

    def heater(self, pct: float) -> None:
        self.set_output(OUT_HEATER, int(round(max(0.0, min(100.0, pct)))))

    def convection(self, enable: bool = True) -> None:
        """Convection (circulation) fan - enable only, no speed (reg 0x36)."""
        self.set_output(CONVECTION, 1 if enable else 0)

    def cooling_fan(self, speed: float) -> None:
        """Cooling fan speed % (reg 0x35)."""
        self.set_output(COOL_FAN, int(round(max(0.0, min(100.0, speed)))))

    def safe_off(self) -> None:
        """Best-effort: heater off, outputs off. Never raises."""
        for cmd in (OUT_HEATER, OUT_35, OUT_36):
            try:
                self.set_output(cmd, 0)
            except Exception:
                pass

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
