"""Packet validation and decoding helpers."""

from __future__ import annotations

from dataclasses import dataclass
import struct

START_BYTE = 0xAA
END_BYTE = 0xBB
SENSORS_PER_PACKET = 16
AXES_PER_SENSOR = 3
MAGNETIC_FIELD_VALUE_COUNT = SENSORS_PER_PACKET * AXES_PER_SENSOR
TRACKING_OUTPUT_VALUE_COUNT = 6
FLOAT_VALUE_COUNT = MAGNETIC_FIELD_VALUE_COUNT + TRACKING_OUTPUT_VALUE_COUNT
FLOAT_SIZE = 4
PAYLOAD_SIZE = FLOAT_VALUE_COUNT * FLOAT_SIZE
PACKET_SIZE = 1 + PAYLOAD_SIZE + 1


@dataclass(frozen=True)
class SensorReading:
    """One magnetic-field reading containing three axes."""

    bx: float
    by: float
    bz: float


@dataclass(frozen=True)
class PacketData:
    """Decoded data from one board packet."""

    magnetic_readings: list[SensorReading]
    tracking_outputs: tuple[float, ...]


class PacketError(Exception):
    """Raised when a packet is malformed or cannot be decoded."""


def decode_packet(packet_bytes: bytes):
    """Validate and decode one complete packet.

    Expected layout:
    [0xAA][48 magnetic-field floats][6 tracking-output floats][0xBB]

    Each value is encoded as a little-endian 32-bit float.
    """
    if len(packet_bytes) != PACKET_SIZE:
        raise PacketError(
            f"Invalid packet length: expected {PACKET_SIZE}, got {len(packet_bytes)}"
        )

    if packet_bytes[0] != START_BYTE:
        raise PacketError(
            f"Invalid start byte: expected 0x{START_BYTE:02X}, got 0x{packet_bytes[0]:02X}"
        )

    if packet_bytes[-1] != END_BYTE:
        raise PacketError(
            f"Invalid end byte: expected 0x{END_BYTE:02X}, got 0x{packet_bytes[-1]:02X}"
        )

    payload = packet_bytes[1:-1]
    values = struct.unpack("<" + ("f" * FLOAT_VALUE_COUNT), payload)

    readings = []
    for sensor_index in range(SENSORS_PER_PACKET):
        offset = sensor_index * AXES_PER_SENSOR
        readings.append(
            SensorReading(
                bx=values[offset],
                by=values[offset + 1],
                bz=values[offset + 2],
            )
        )

    tracking_outputs = values[MAGNETIC_FIELD_VALUE_COUNT:]
    return PacketData(magnetic_readings=readings, tracking_outputs=tracking_outputs)
