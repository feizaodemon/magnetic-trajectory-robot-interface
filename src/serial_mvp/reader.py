"""Byte-stream reader for extracting complete packets."""

from __future__ import annotations

from .packet import PACKET_SIZE, START_BYTE, PacketError, decode_packet


class SerialPacketReader:
    """Read bytes continuously and yield validated packets."""

    def __init__(
        self,
        byte_source,
        read_size: int = 4096,
        max_packets: int | None = None,
        packet_handler=None,
        diagnostics=None,
        verbose_packets: bool = False,
        max_buffer_bytes: int = PACKET_SIZE * 100,
    ):
        self.byte_source = byte_source
        self.read_size = read_size
        self.buffer = bytearray()
        self.max_packets = max_packets
        self.packet_handler = packet_handler
        self.decoded_packets = 0
        self.diagnostics = diagnostics
        self.verbose_packets = verbose_packets
        self.max_buffer_bytes = max_buffer_bytes
        self.malformed_packets = 0

    def _discard_until_start_byte(self):
        """Remove noise bytes until the next possible packet start."""
        start_index = self.buffer.find(bytes([START_BYTE]))
        if start_index == 0:
            return

        if start_index == -1:
            dropped_count = len(self.buffer)
            self.buffer.clear()
        else:
            dropped_count = start_index
            del self.buffer[:start_index]

        if dropped_count > 0 and self.diagnostics is not None:
            self.diagnostics.add_noise_bytes(dropped_count)

    def _process_buffer(self):
        """Extract as many complete packets as possible from the buffer."""
        self._discard_until_start_byte()

        while len(self.buffer) >= PACKET_SIZE:
            if self.max_packets is not None and self.decoded_packets >= self.max_packets:
                return

            candidate = bytes(self.buffer[:PACKET_SIZE])
            try:
                packet_data = decode_packet(candidate)
            except PacketError as exc:
                self.malformed_packets += 1
                if self.diagnostics is not None:
                    self.diagnostics.add_malformed_packet()
                if self.verbose_packets:
                    print(f"[WARN] Invalid packet discarded: {exc}")
                # Drop the current start byte and rescan to recover alignment.
                del self.buffer[0]
                self._discard_until_start_byte()
                continue

            del self.buffer[:PACKET_SIZE]
            self.decoded_packets += 1
            if self.diagnostics is not None:
                self.diagnostics.add_parsed_packet()
            if self.verbose_packets:
                self._print_packet(packet_data)
            if self.packet_handler is not None:
                self.packet_handler(self.decoded_packets, packet_data)
            self._discard_until_start_byte()

    def should_stop(self) -> bool:
        """Return True when the requested number of packets has been decoded."""
        return self.max_packets is not None and self.decoded_packets >= self.max_packets

    def read_once(self):
        """Read all currently available bytes and process complete packets."""
        if self.should_stop():
            return

        waiting_bytes = self._available_bytes()
        if waiting_bytes is None:
            incoming = self.byte_source.read(self.read_size)
            if incoming:
                self._accept_bytes(incoming)
            return

        if waiting_bytes <= 0:
            incoming = self.byte_source.read(self.read_size)
            if incoming:
                self._accept_bytes(incoming)
            return

        while waiting_bytes > 0 and not self.should_stop():
            incoming = self.byte_source.read(waiting_bytes)
            if not incoming:
                return
            self._accept_bytes(incoming)
            next_waiting_bytes = self._available_bytes()
            if next_waiting_bytes is None:
                return
            waiting_bytes = next_waiting_bytes

    def _accept_bytes(self, incoming: bytes):
        """Append incoming bytes, trim runaway buffers, and parse packets."""
        if self.diagnostics is not None:
            self.diagnostics.add_received_bytes(len(incoming))

        # Bytes may arrive in arbitrary chunk sizes, so keep incomplete packets
        # in the buffer until enough data arrives in a later read cycle.
        self.buffer.extend(incoming)
        if len(self.buffer) > self.max_buffer_bytes:
            excess_byte_count = len(self.buffer) - self.max_buffer_bytes
            del self.buffer[:excess_byte_count]
            self.malformed_packets += 1
            if self.diagnostics is not None:
                self.diagnostics.add_malformed_packet()
                self.diagnostics.add_noise_bytes(excess_byte_count)
        self._process_buffer()

    def _available_bytes(self) -> int | None:
        try:
            return int(getattr(self.byte_source, "in_waiting"))
        except AttributeError:
            return None

    def _print_packet(self, packet_data):
        print(f"[INFO] Decoded packet #{self.decoded_packets}")
        for sensor_index, reading in enumerate(packet_data.magnetic_readings, start=1):
            print(
                f"  S{sensor_index:02d}: "
                f"Bx={reading.bx:.3f} By={reading.by:.3f} Bz={reading.bz:.3f}"
            )
        tracking_values = " ".join(
            f"T{index}={value:.3f}"
            for index, value in enumerate(packet_data.tracking_outputs, start=1)
        )
        print(f"  Tracking: {tracking_values}")

    def run(self):
        """Continuously read, validate, and decode packets."""
        while True:
            if self.should_stop():
                return

            self.read_once()
