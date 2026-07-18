"""Real and simulated byte sources for the serial MVP."""

from __future__ import annotations

import itertools
import struct
import time

import serial

from .packet import (
    AXES_PER_SENSOR,
    END_BYTE,
    FLOAT_VALUE_COUNT,
    SENSORS_PER_PACKET,
    START_BYTE,
)


class SerialPortSource:
    """Read bytes from a real serial port using pyserial."""

    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

    def open(self):
        """Open and return the serial connection."""
        return serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )


class SimulatedStream:
    """Minimal stream-like object that returns bytes in small chunks."""

    def __init__(self, chunks: list[bytes], delay_seconds: float = 0.1):
        self._chunks = itertools.cycle(chunks)
        self._delay_seconds = delay_seconds

    def read(self, size: int = 1) -> bytes:
        """Return the next chunk of simulated bytes.

        The size argument is accepted to match pyserial's API, but the stream
        intentionally returns protocol-sized fragments to test buffer handling.
        """
        del size
        time.sleep(self._delay_seconds)
        return next(self._chunks)

    def close(self):
        """Match the serial.Serial interface used by the main entry point."""
        return None


class SimulatedSource:
    """Provide test bytes without requiring real hardware."""

    def __init__(self, delay_seconds: float = 0.1):
        self.delay_seconds = delay_seconds

    @staticmethod
    def _make_valid_packet(base_value: float) -> bytes:
        values = []
        for sensor_index in range(SENSORS_PER_PACKET):
            sensor_base = base_value + sensor_index
            values.extend(
                [
                    sensor_base + 0.1,
                    sensor_base + 0.2,
                    sensor_base + 0.3,
                ]
            )
        values.extend([base_value + 100.0 + output_index for output_index in range(6)])

        payload = struct.pack(
            "<" + ("f" * FLOAT_VALUE_COUNT),
            *values,
        )
        return bytes([START_BYTE]) + payload + bytes([END_BYTE])

    @staticmethod
    def _make_invalid_packet() -> bytes:
        valid_packet = SimulatedSource._make_valid_packet(100.0)
        payload = valid_packet[1:-1]
        return bytes([START_BYTE]) + payload + bytes([0x00])

    def open(self):
        """Return a simulated byte source."""
        packet_a = self._make_valid_packet(10.0)
        packet_b = self._make_valid_packet(-5.0)
        packet_c = self._make_valid_packet(0.0)

        chunks = [
            b"\x00\x01",  # Noise before the first valid packet.
            packet_a[:40],
            packet_a[40:],
            self._make_invalid_packet(),
            packet_b[:75],
            packet_b[75:],
            packet_c[:90],
            packet_c[90:],
        ]
        return SimulatedStream(chunks, delay_seconds=self.delay_seconds)
