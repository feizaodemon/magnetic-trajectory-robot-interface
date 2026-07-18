"""Checks for byte buffering and packet resynchronization."""

from __future__ import annotations

from collections import deque
import struct

from serial_mvp.packet import END_BYTE, FLOAT_VALUE_COUNT, START_BYTE
from serial_mvp.reader import SerialPacketReader


def make_valid_packet(base_value: float) -> bytes:
    values = [base_value + index for index in range(FLOAT_VALUE_COUNT)]
    payload = struct.pack("<" + ("f" * FLOAT_VALUE_COUNT), *values)
    return bytes([START_BYTE]) + payload + bytes([END_BYTE])


def make_invalid_packet() -> bytes:
    packet = bytearray(make_valid_packet(100.0))
    packet[-1] = END_BYTE - 1
    return bytes(packet)


class ChunkStream:
    """Small byte source that returns predefined chunks."""

    def __init__(self, chunks: list[bytes]):
        self.chunks = deque(chunks)

    def read(self, size: int = 1) -> bytes:
        del size
        if not self.chunks:
            return b""
        return self.chunks.popleft()


class WaitingStream:
    """Byte source that reports how many bytes are available."""

    def __init__(self, data: bytes):
        self.data = bytearray(data)

    @property
    def in_waiting(self) -> int:
        return len(self.data)

    def read(self, size: int = 1) -> bytes:
        chunk = bytes(self.data[:size])
        del self.data[:size]
        return chunk


def collect_packets(reader: SerialPacketReader) -> list:
    packets = []
    reader.packet_handler = lambda packet_number, packet_data: packets.append(
        (packet_number, packet_data)
    )
    return packets


def test_complete_packet_is_parsed():
    reader = SerialPacketReader(ChunkStream([make_valid_packet(1.0)]))
    packets = collect_packets(reader)

    reader.read_once()

    assert len(packets) == 1
    assert reader.decoded_packets == 1


def test_multiple_packets_in_one_read_are_parsed():
    packet_a = make_valid_packet(1.0)
    packet_b = make_valid_packet(2.0)
    reader = SerialPacketReader(ChunkStream([packet_a + packet_b]))
    packets = collect_packets(reader)

    reader.read_once()

    assert len(packets) == 2
    assert reader.decoded_packets == 2


def test_all_waiting_bytes_are_read_in_one_cycle():
    packet_a = make_valid_packet(1.0)
    packet_b = make_valid_packet(2.0)
    reader = SerialPacketReader(WaitingStream(packet_a + packet_b))
    packets = collect_packets(reader)

    reader.read_once()

    assert len(packets) == 2
    assert reader.byte_source.in_waiting == 0


def test_incomplete_packet_is_preserved_across_reads():
    packet = make_valid_packet(3.0)
    reader = SerialPacketReader(ChunkStream([packet[:30], packet[30:]]))
    packets = collect_packets(reader)

    reader.read_once()
    assert packets == []

    reader.read_once()
    assert len(packets) == 1
    assert reader.buffer == bytearray()


def test_malformed_packet_is_counted_and_discarded():
    reader = SerialPacketReader(ChunkStream([make_invalid_packet()]))
    packets = collect_packets(reader)

    reader.read_once()

    assert packets == []
    assert reader.malformed_packets == 1


def test_valid_packet_after_malformed_data_is_parsed():
    malformed = bytearray(make_valid_packet(4.0))
    malformed[-1] = END_BYTE - 1
    valid = make_valid_packet(5.0)
    reader = SerialPacketReader(ChunkStream([bytes(malformed) + valid]))
    packets = collect_packets(reader)

    reader.read_once()

    assert len(packets) == 1
    assert reader.decoded_packets == 1
    assert reader.malformed_packets == 1


if __name__ == "__main__":
    test_complete_packet_is_parsed()
    test_multiple_packets_in_one_read_are_parsed()
    test_all_waiting_bytes_are_read_in_one_cycle()
    test_incomplete_packet_is_preserved_across_reads()
    test_malformed_packet_is_counted_and_discarded()
    test_valid_packet_after_malformed_data_is_parsed()
