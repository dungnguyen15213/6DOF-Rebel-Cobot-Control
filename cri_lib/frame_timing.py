"""Provenance of complete CRI STATUS frames extracted from TCP reads.

CRI STATUS messages carry no sample timestamp or sequence number (the ``Time``
field exists only in CAN-bridge messages), so the only trustworthy clock is the
host monotonic clock. One ``recv()`` may deliver several complete frames; those
frames share a single receive time and were sampled earlier at unknown instants.
Nothing here invents spacing between them. It only labels them.
"""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

TIMING_UNIQUE = "UNIQUE"
TIMING_BATCH_LAST = "BATCH_LAST"
TIMING_BATCH_EARLIER = "BATCH_EARLIER"

_START = b"CRISTART"
_END = b"CRIEND"


@dataclass(frozen=True)
class FrameTiming:
    """Timing provenance of one complete frame.

    ``t_chunk_rx_s`` is ``perf_counter()`` right after the ``recv()`` that
    completed the frame. ``t_frame_s`` is ``perf_counter()`` when the frame
    started processing; within a batch it only reflects parse order, not
    acquisition time. ``frame_counter``/``chunk_id`` restart on every
    connection, so use them together with ``connection_epoch``.
    """

    connection_epoch: int
    frame_counter: int
    chunk_id: int
    frame_in_chunk: int
    frames_in_chunk: int
    t_chunk_rx_s: float
    t_frame_s: float

    @property
    def timing_class(self) -> str:
        if self.frames_in_chunk <= 1:
            return TIMING_UNIQUE
        if self.frame_in_chunk == self.frames_in_chunk - 1:
            return TIMING_BATCH_LAST
        return TIMING_BATCH_EARLIER

    @property
    def usable_for_derivatives(self) -> bool:
        return self.timing_class != TIMING_BATCH_EARLIER


def extract_frames(buffer: bytearray) -> tuple[list[str], int]:
    """Remove every complete frame from ``buffer`` and return ``(frames, discarded)``.

    Incomplete trailing data stays in the buffer for the next read. A terminator
    without a start marker, or undecodable bytes, is discarded and counted
    instead of raising. This keeps the previous framing rule (one separator byte
    after ``CRIEND`` is consumed).
    """
    frames: list[str] = []
    discarded = 0
    while True:
        end_idx = buffer.find(_END)
        if end_idx == -1:
            break
        start_idx = buffer.find(_START)
        if start_idx != -1 and start_idx < end_idx:
            try:
                frames.append(bytes(buffer[start_idx : end_idx + 6]).decode())
            except UnicodeDecodeError:
                discarded += 1
        else:
            discarded += 1
        if len(buffer) > end_idx + 7:
            del buffer[: end_idx + 7]
        else:
            buffer.clear()
    return frames, discarded


class FrameTimer:
    """Assigns provenance to frames; not thread-safe (receive thread only)."""

    def __init__(self) -> None:
        self.connection_epoch = 0
        self.frame_counter = 0
        self.chunk_id = 0
        self.discarded_frames = 0

    def restart_connection(self) -> None:
        self.connection_epoch += 1
        self.frame_counter = 0
        self.chunk_id = 0

    def begin_chunk(self) -> int:
        chunk_id = self.chunk_id
        self.chunk_id += 1
        return chunk_id

    def stamp_frame(
        self, chunk_id: int, frame_in_chunk: int, frames_in_chunk: int, t_chunk_rx_s: float
    ) -> FrameTiming:
        """Build provenance when the frame starts processing (sets ``t_frame_s``)."""
        timing = FrameTiming(
            connection_epoch=self.connection_epoch,
            frame_counter=self.frame_counter,
            chunk_id=chunk_id,
            frame_in_chunk=frame_in_chunk,
            frames_in_chunk=frames_in_chunk,
            t_chunk_rx_s=t_chunk_rx_s,
            t_frame_s=perf_counter(),
        )
        self.frame_counter += 1
        return timing