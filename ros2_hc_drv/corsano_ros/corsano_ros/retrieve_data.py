from corsano_ros.corsano_driver import CorsanoDriver
from corsano_ros.parsers.activity_parser import ActivityParser, ActivityData
from corsano_ros.parsers.bioz_parser import BioZParser, BioZData
from corsano_ros.parsers.stress_parser import StressParser, StressData
from corsano_ros.parsers.accelerometer_parser import AccelerometerParser, AccelerometerData
from corsano_ros.corsano_enums import FileNames
from corsano_ros.commands import Command
from logging import debug
import time
from typing import Optional
import sys

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

    return activity

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

    return last_packet


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
    return stress


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
        return parser.process_metric_array(buffer_data, 0, metric_id=0x2B, metric_size=len(buffer_data))
    except Exception as e:
        debug(f"[corsano_ros::get_last_accelerometer_data] Failed to parse accelerometer data: {e}")
        return None
