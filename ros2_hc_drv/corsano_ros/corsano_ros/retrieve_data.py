from corsano_ros.corsano_driver import CorsanoDriver
from corsano_ros.parsers.activity_parser import ActivityParser, ActivityData
from corsano_ros.parsers.bioz_parser import BioZParser, BioZData
from corsano_ros.parsers.stress_parser import StressParser, StressData
from corsano_ros.parsers.accelerometer_parser import AccelerometerParser, AccelerometerData
from corsano_ros.corsano_enums import FileNames
from corsano_ros.commands import Command
from logging import debug, info, warning
import time
from typing import Optional
import sys


# Last file size published per watch file. The watch only appends to these
# files, so an unchanged size means nothing new was recorded. Without this
# the driver re-downloaded the same unchanged tail on every poll and
# published it as a fresh message each time -- confirmed 2026-10-01: with no
# skin contact the ACC/BioZ/Stress files didn't grow for days, yet
# /corsano/acceleration, /bioz and /stress each got ~75 identical messages
# per 5 min, so the topics looked live. Polling and downloads are left
# exactly as they were (only publishing is gated), so the BLE traffic the
# wireless experiments measure is unchanged.
_last_size = {}
_last_tail = {}
_stale_since = {}
STALE_WARNING_S = 60.0

# TEST ONLY -- set by the wrapper's --test-republish-latest flag for the
# wireless experiments (scai_ws/src/wireless_experiments), never on by
# default. Streams named here are published on EVERY poll with the latest
# record the watch holds, even when nothing new was recorded -- the old
# behaviour -- so the topics carry a steady message load during a link test
# regardless of whether the watch is worn. Those messages can be stale; do
# NOT use them as sensor data. Freshness is still tracked and the
# "no new data" warning still logged.
TEST_REPUBLISH_LATEST = set()


def _publishable(name, size, data, tail):
    """Return data only if the file's downloaded tail changed since the last
    poll that got this far; None otherwise. The first poll after startup
    just records it: that tail may have been written long before the driver
    started, so it isn't published as current.

    Compares the tail bytes, not the file size: the watch's files are
    circular once its storage is full -- seen 2026-10-01, BioZ grew
    3886017 -> 3887047 then dropped to 3883260 as the oldest block was
    deleted -- so a size change alone can be a deletion with nothing new,
    and a full file could take new data without changing size."""
    prev = _last_tail.get(name)
    prev_size = _last_size.get(name)
    _last_tail[name] = tail
    _last_size[name] = size
    now = time.monotonic()
    # TEST ONLY (see TEST_REPUBLISH_LATEST): publish regardless, but keep
    # the freshness bookkeeping/warnings below.
    result_if_stale = data if name in TEST_REPUBLISH_LATEST else None
    if prev is None:
        _stale_since[name] = now
        if result_if_stale is None:
            info(f"[corsano_ros] {name}: starting at {size} bytes; publishing only data recorded from now on")
        return result_if_stale
    if tail == prev:
        since = _stale_since.setdefault(name, now)
        if since is not None and now - since >= STALE_WARNING_S:
            warning(
                f"[corsano_ros] {name}: no new data for {now - since:.0f}s (file stays at "
                f"{size} bytes) -- the watch isn't recording it; is it worn with skin contact?"
            )
            _stale_since[name] = None  # warn once per stale period
        return result_if_stale
    if _stale_since.get(name, 0) is None:
        info(f"[corsano_ros] {name}: new data again ({prev_size} -> {size} bytes)")
    _stale_since[name] = now
    return data

def get_last_activity_data(
    driver: CorsanoDriver,
    cmd_get_file_size: Command,
    cmd_stream_file_with_size: Command,
    cmd_stream_file_with_size_offset: Command,
) -> Optional[ActivityData]:
    try:
        file = driver.execute(cmd_get_file_size.cmd, file=FileNames["Activity_file"])
        size: int = file["size"]
    except Exception as e:
        debug(f"[corsano_ros::get_last_activity_data] Failed to get Activity file size: {e}")
        return None

    offset: int = size - 38

    if offset < 0:
        driver.execute(cmd_stream_file_with_size.cmd, file=FileNames["Activity_file"], size=38)
    else:
        driver.execute(
            cmd_stream_file_with_size_offset.cmd, file=FileNames["Activity_file"], size=38, offset=offset
        )

    time.sleep(1)
    buffer_data: bytes = driver.get_buffer().read()
    try:
        activity: Optional[ActivityData] = ActivityParser.parse(buffer_data)
    except Exception as e:
        debug(f"[corsano_ros::get_last_activity_data] Failed to parse Activity data: {e}")
        return None

    if activity is None:
        debug("[corsano_ros::get_last_activity_data] CRC check failed or invalid data")
        return None

    return _publishable("Activity", size, activity, bytes(buffer_data))

def dump_bioz_file(
    driver: CorsanoDriver,
    cmd_get_file_size: Command,
    cmd_stream_file_with_size: Command,
    cmd_stream_file_with_size_offset: Command,
    output_path: str,
    file_name: str = FileNames.BioZ_file,
    chunk_size: int = 2048,
) -> bool:
    """
    Download the entire BioZ file from the device and write it to a local file.
    Returns True on success.
    """

    # --- Get total size ---
    try:
        file_info = driver.execute(cmd_get_file_size.cmd, file=file_name)
        size: int = file_info["size"]
        debug(f"BioZ file size: {size} bytes")
    except Exception as e:
        debug(f"[dump_bioz_file] Failed to get BioZ file size: {e}")
        return False

    if size == 0:
        debug("[dump_bioz_file] BioZ file is empty.")
        return False

    # --- Open local file ---
    with open(output_path, "wb") as f_out:

        bytes_downloaded = 0

        while bytes_downloaded < size:
            try:
                remaining = size - bytes_downloaded
                read_size = min(chunk_size, remaining)

                # choose correct command
                if bytes_downloaded == 0:
                    driver.execute(
                        cmd_stream_file_with_size.cmd,
                        file=file_name,
                        size=read_size
                    )
                else:
                    driver.execute(
                        cmd_stream_file_with_size_offset.cmd,
                        file=file_name,
                        size=read_size,
                        offset=bytes_downloaded
                    )

                # give the buffer time to fill
                time.sleep(0.1)
                data = driver.get_buffer().read()

                if not data:
                    debug("[dump_bioz_file] No data returned from device. Retrying...")
                else:

                    f_out.write(data)
                    bytes_downloaded += len(data)

                    debug(f"Downloaded {bytes_downloaded}/{size} bytes")
            except Exception as e:
                debug(f"[dump_bioz_file] Error downloading the file, retrying... {e}")
        debug(f"[dump_bioz_file] File successfully saved to: {output_path}")
        sys.exit()


def get_last_bioz_data(
    driver: CorsanoDriver,
    cmd_get_file_size: Command,
    cmd_stream_file_with_size: Command,
    cmd_stream_file_with_size_offset: Command,
    file_name: str = FileNames.BioZ_file,
    chunk_size: int = 2048,
) -> Optional[BioZData]:
    """
    Fetch the last portion of the BioZ file and decode the last BioZ record.
    Returns a BioZData object containing record_index, quality, and 25 measurements.
    """
    parser = BioZParser()

    # --- Get file size ---
    try:
        file_info = driver.execute(cmd_get_file_size.cmd, file=file_name)
        size: int = file_info["size"]
        debug(f"BioZ file size: {size} bytes")
    except Exception as e:
        debug(f"[get_last_bioz_data] Failed to get BioZ file size: {e}")
        return None

    if size == 0:
        debug("[get_last_bioz_data] BioZ file is empty.")
        return None

    # --- Download last chunk of file ---
    offset = max(0, size - chunk_size)
    read_size = min(chunk_size, size - offset)
    try:
        if offset == 0:
            driver.execute(cmd_stream_file_with_size.cmd, file=file_name, size=read_size)
        else:
            driver.execute(
                cmd_stream_file_with_size_offset.cmd,
                file=file_name,
                size=read_size,
                offset=offset,
            )
    except Exception as e:
        debug(f"[get_last_bioz_data] Failed to stream BioZ file: {e}")
        return None

    time.sleep(0.5)  # allow buffer to fill
    buffer_data = driver.get_buffer().read()

    if not buffer_data:
        debug("[get_last_bioz_data] No data returned from buffer.")
        return None

    # --- Parse packets and get last ---
    try:
        parser.parse_packets(buffer_data)
        last_packet = parser.get_last_packet()
    except Exception as e:
        debug(f"[get_last_bioz_data] Failed to parse BioZ data: {e}")
        return None
    if last_packet is None:
        debug("[get_last_bioz_data] No BioZ packets found in buffer.")
        return None

    debug(
        f"[get_last_bioz_data] Decoded last BioZ record: "
        f"timestamp={last_packet.timestamp}, "
        f"index={last_packet.record_index}, "
        f"measurements={last_packet.values}"
    )

    return _publishable("BioZ", size, last_packet, bytes(buffer_data))


def get_last_stress_data(
    driver: CorsanoDriver,
    cmd_get_file_size: Command,
    cmd_stream_file_with_size: Command,
    cmd_stream_file_with_size_offset: Command,
    file_name: str = FileNames["STRESS_FILE"],
) -> Optional[StressData]:
    parser = StressParser()

    try:
        file = driver.execute(cmd_get_file_size.cmd, file=file_name)
        size: int = file["size"]
    except Exception as e:
        debug(f"[corsano_ros::get_last_stress_data] Failed to get Stress file size: {e}")
        return None

    if size == 0:
        debug("[corsano_ros::get_last_stress_data] Stress file is empty — check measurement plan.")
        return None

    offset: int = max(size - 18, 0)

    try:
        driver.execute(cmd_stream_file_with_size_offset.cmd, file=file_name, size=18, offset=offset)
    except Exception as e:
        debug(f"[corsano_ros::get_last_stress_data] Failed to stream Stress file: {e}")
        return None

    time.sleep(1)
    buffer_data: bytes = driver.get_buffer().read()
    if not buffer_data or len(buffer_data) < 18:
        debug("[corsano_ros::get_last_stress_data] No data returned or not enough bytes.")
        return None

    try:
        stress = parser.parse_last_record(buffer_data)
    except Exception as e:
        debug(f"[corsano_ros::get_last_stress_data] Failed to parse Stress data: {e}")
        return None
    debug(stress)
    return _publishable("Stress", size, stress, bytes(buffer_data))


def get_last_accelerometer_data(
    driver: CorsanoDriver,
    cmd_get_file_size: Command,
    cmd_stream_file_with_size: Command,
    cmd_stream_file_with_size_offset: Command,
    file_name: str = FileNames["ACC_FILE"],
) -> Optional[AccelerometerData]:
    parser = AccelerometerParser()

    try:
        file = driver.execute(cmd_get_file_size.cmd, file=file_name)
        size: int = file["size"]
    except Exception as e:
        debug(f"[corsano_ros::get_last_accelerometer_data] Failed to get accelerometer file size: {e}")
        return None

    if size == 0:
        debug("[corsano_ros::get_last_accelerometer_data] Accelerometer file is empty — check measurement plan.")
        return None

    offset: int = max(size - 1024, 0)

    try:
        if offset == 0:
            driver.execute(cmd_stream_file_with_size.cmd, file=file_name, size=size)
        else:
            driver.execute(
                cmd_stream_file_with_size_offset.cmd, file=file_name, size=size, offset=offset
            )
    except Exception as e:
        debug(f"[corsano_ros::get_last_accelerometer_data] Failed to stream accelerometer file: {e}")
        return None

    time.sleep(1)
    buffer_data: bytes = driver.get_buffer().read()
    if not buffer_data:
        debug("[corsano_ros::get_last_accelerometer_data] No data returned from buffer.")
        return None

    try:
        acc = parser.process_metric_array(buffer_data, 0, metric_id=0x2B, metric_size=len(buffer_data))
    except Exception as e:
        debug(f"[corsano_ros::get_last_accelerometer_data] Failed to parse accelerometer data: {e}")
        return None
    return _publishable("ACC", size, acc, bytes(buffer_data))
