from __future__ import annotations
from logging import warning
from dataclasses import dataclass
from datetime import timezone
from typing import Optional
import numpy as np


@dataclass
class AccelerometerData:
    """
    Container for parsed accelerometer data from a Corsano device.

    Attributes:
        index: Packet counter (0–255).
        quality: Data quality flag.
        bpi: Body position index.
        af: Accelerometer format.
        x_values: Array of X-axis accelerometer readings (m/s²).
        y_values: Array of Y-axis accelerometer readings (m/s²).
        z_values: Array of Z-axis accelerometer readings (m/s²).
        timestamp_ms: Timestamp of the first sample in milliseconds.
    """

    index: int
    quality: int
    bpi: int
    af: int
    x_values: np.ndarray
    y_values: np.ndarray
    z_values: np.ndarray
    timestamp_ms: float


class AccelerometerParser:
    """
    Parser for accelerometer metric arrays received from Corsano devices.

    This class decodes raw binary accelerometer packets into structured
    arrays of acceleration data, converting to SI units (m/s²).

    Wire format, confirmed against a live device (records are 205 bytes,
    packed back-to-back in the downloaded chunk with no guarantee that the
    chunk starts on a record boundary):
        [0:5]   fixed magic: 48 52 c8 00 0f
        [5]     metric id (0x2B for accelerometer)
        [6:8]   payload length, little-endian (bytes following this field)
        [8]     packet index (0-255, wraps)
        [9]     quality
        [10]    bpi
        [11]    af
        [12]    reserved
        [13:]   payload_length - 4 bytes of samples, 6 bytes each:
                X, Y, Z as big-endian signed 16-bit integers
    """

    ACC_SR: int = 32  # Sampling rate in Hz (samples per second)
    MAGIC_PREFIX = bytes.fromhex("4852c8000f")
    HEADER_LEN = 13  # magic(6) + length(2) + index(1) + quality/bpi/af/reserved(4)
    SAMPLE_SIZE = 6  # 3 axes x 2 bytes, big-endian signed

    def __init__(self, time_zone: timezone = timezone.utc) -> None:
        self.acc_time: float = 0.0
        self.time_zone: timezone = time_zone
        self.normalize: bool = False
        self.last_index: Optional[int] = None

    def process_metric_array(
        self,
        metric_array: bytes,
        processed_index: int,
        metric_id: int,
        metric_size: int
    ) -> Optional[AccelerometerData]:
        """
        Scan a downloaded chunk for accelerometer records and return the
        most recent one found.

        The chunk isn't guaranteed to start on a record boundary (it's a
        byte-offset window into the device's file, not one self-contained
        record), so this synchronizes on the record's magic prefix rather
        than assuming processed_index/metric_size delimit a single record.

        Returns the last successfully decoded AccelerometerData, or None if
        no complete, recognized record was found.
        """
        if metric_id != 0x2B:
            warning(
                f"[AccelerometerParser] Skipping unsupported metric ID: "
                f"0x{metric_id:02X} ({metric_id})"
            )
            return None

        last_result = None
        i = processed_index
        n = len(metric_array)

        while i < n:
            idx = metric_array.find(self.MAGIC_PREFIX, i)
            if idx == -1:
                break
            if idx + 6 >= n or metric_array[idx + 5] != metric_id:
                i = idx + 1
                continue
            if idx + self.HEADER_LEN > n:
                break

            payload_len = int.from_bytes(
                metric_array[idx + 6 : idx + 8], byteorder="little"
            )
            record_len = 8 + payload_len  # length field covers everything after it
            if idx + record_len > n:
                break

            record = self._decode_record(metric_array, idx, payload_len)
            if record is not None:
                last_result = record

            i = idx + record_len

        return last_result

    def _decode_record(
        self, metric_array: bytes, start: int, payload_len: int
    ) -> Optional[AccelerometerData]:
        """Decode one record beginning at `start` into structured data."""
        index = metric_array[start + 8]
        quality = metric_array[start + 9]
        bpi = metric_array[start + 10]
        af = metric_array[start + 11]

        # Detect dropped packets
        if self.last_index is not None:
            expected = (self.last_index + 1) % 256
            if index != expected:
                warning(
                    f"[AccelerometerParser] Packet drop detected: "
                    f"expected {expected}, got {index}"
                )
        self.last_index = index

        samples_start = start + self.HEADER_LEN
        num_samples = (payload_len - 4) // self.SAMPLE_SIZE
        if num_samples <= 0:
            return None

        scale = 9.80665 / 512.0  # Convert raw units to m/s²
        x_values = np.empty(num_samples, dtype=np.float32)
        y_values = np.empty(num_samples, dtype=np.float32)
        z_values = np.empty(num_samples, dtype=np.float32)

        offset = samples_start
        for i in range(num_samples):
            x_values[i] = int.from_bytes(
                metric_array[offset:offset + 2], byteorder="big", signed=True
            ) * scale
            y_values[i] = int.from_bytes(
                metric_array[offset + 2:offset + 4], byteorder="big", signed=True
            ) * scale
            z_values[i] = int.from_bytes(
                metric_array[offset + 4:offset + 6], byteorder="big", signed=True
            ) * scale
            offset += self.SAMPLE_SIZE

        data = AccelerometerData(
            index=index,
            quality=quality,
            bpi=bpi,
            af=af,
            x_values=x_values,
            y_values=y_values,
            z_values=z_values,
            timestamp_ms=self.acc_time,
        )
        self.acc_time += num_samples * 1000.0 / self.ACC_SR
        return data

    def reset(self) -> None:
        """Reset parser timestamp and packet counter."""
        self.acc_time = 0.0
        self.last_index = None
