from __future__ import annotations

from dataclasses import dataclass


HEADER = b"\xCF\xFA"
MIN_DISTANCE_MM = 50
MAX_DISTANCE_MM = 12000
MIN_POINTS = 1
MAX_POINTS = 200
FRAME_PREFIX_SIZE = 8
FRAME_TRAILER_SIZE = 2


@dataclass(frozen=True)
class LidarPoint:
    angle_deg: float
    distance_mm: int
    quality: int
    valid: bool = True


@dataclass(frozen=True)
class CffaPacket:
    points: list[LidarPoint]
    all_points: list[LidarPoint]
    start_angle_deg: float
    angle_span_deg: float
    trailer: bytes
    raw_size: int


def _u16_le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def find_header(buffer: bytes, start: int = 0) -> int:
    return buffer.find(HEADER, start)


def parse_packet_at(buffer: bytes, offset: int) -> tuple[CffaPacket | None, int]:
    """Intenta parsear un paquete CF FA experimental desde offset.

    El formato todavia no esta confirmado para el PACECAT LDS-E120-R. Esta
    rutina valida longitudes y rangos para evitar interpretar ruido como datos.
    """
    if offset < 0:
        return None, offset + 1
    if buffer[offset : offset + 2] != HEADER:
        return None, offset + 1
    if offset + FRAME_PREFIX_SIZE > len(buffer):
        return None, offset

    point_count = _u16_le(buffer, offset + 2)
    if not (MIN_POINTS <= point_count <= MAX_POINTS):
        return None, offset + 2

    frame_size = FRAME_PREFIX_SIZE + point_count * 3 + FRAME_TRAILER_SIZE
    if offset + frame_size > len(buffer):
        return None, offset

    start_angle_raw = _u16_le(buffer, offset + 4)
    angle_span_raw = _u16_le(buffer, offset + 6)
    start_angle_deg = (start_angle_raw / 10.0) % 360.0
    angle_span_deg = angle_span_raw / 10.0

    points: list[LidarPoint] = []
    all_points: list[LidarPoint] = []
    point_base = offset + FRAME_PREFIX_SIZE

    for index in range(point_count):
        point_offset = point_base + index * 3
        quality = buffer[point_offset]
        distance_mm = _u16_le(buffer, point_offset + 1)

        angle_deg = (start_angle_deg + angle_span_deg * index / point_count) % 360.0
        valid = MIN_DISTANCE_MM <= distance_mm <= MAX_DISTANCE_MM
        point = LidarPoint(
            angle_deg=angle_deg,
            distance_mm=distance_mm,
            quality=quality,
            valid=valid,
        )
        all_points.append(point)
        if valid:
            points.append(point)

    trailer_start = offset + FRAME_PREFIX_SIZE + point_count * 3

    return (
        CffaPacket(
            points=points,
            all_points=all_points,
            start_angle_deg=start_angle_deg,
            angle_span_deg=angle_span_deg,
            trailer=buffer[trailer_start : offset + frame_size],
            raw_size=frame_size,
        ),
        offset + frame_size,
    )


def parse_buffer(buffer: bytes) -> tuple[list[CffaPacket], bytes]:
    packets: list[CffaPacket] = []
    cursor = 0

    while cursor < len(buffer):
        header_offset = find_header(buffer, cursor)
        if header_offset == -1:
            keep = buffer[-1:] if buffer.endswith(HEADER[:1]) else b""
            return packets, keep

        packet, next_offset = parse_packet_at(buffer, header_offset)
        if packet is None:
            if next_offset == header_offset:
                return packets, buffer[header_offset:]
            cursor = max(next_offset, header_offset + 1)
            continue

        packets.append(packet)
        cursor = next_offset

    return packets, b""
