from __future__ import annotations

import abc
import hashlib
import struct
from datetime import datetime

from corsano_ros.corsano_enums import (
    FileNames,
    ModeID,
    CHG_STATUS,
    PLAN,
    PLAN_FREQUENCY,
    SPECIAL_MODES,
)


# =============================================================================
# Base Command Classes
# =============================================================================


class BaseCommand(metaclass=abc.ABCMeta):
    """Abstract base class for all command types."""

    cmd = int()
    sidechannel = False


class Command(BaseCommand):
    """Base class for normal (non-HCI) commands."""

    def execute(self):
        """Default execute implementation."""
        return self.cmd.to_bytes(1, "little")


# =============================================================================
# Device Command Implementations
# =============================================================================


class CMD_START_STREAMING_DATA(Command):
    """Send the currently selected file."""

    cmd = 1
    sidechannel = True


class CMD_SET_NORMAL_MODE(Command):
    """Set the watch to normal mode."""

    cmd = 15

    def process(self, data):
        return {"result": data[1]}


class APP_CMD_UPDATE_TIME(Command):
    """Set current time on the watch.

    Arguments:
        year -- Year to set
        month -- Month to set
        day -- Day to set
        hour -- Hour to set
        minute -- Minute to set
        seconds -- Seconds to set
        utc_offset -- Offset in minutes from UTC
    """

    cmd = 22

    def execute(
        self,
        year: int,
        month: int,
        day: int,
        hour: int,
        minute: int,
        seconds: int,
        utc_offset: int,
    ):
        return (
            self.cmd.to_bytes(1, "little")
            + (year - 2000).to_bytes(1, "little")
            + month.to_bytes(1, "little")
            + day.to_bytes(1, "little")
            + hour.to_bytes(1, "little")
            + minute.to_bytes(1, "little")
            + seconds.to_bytes(1, "little")
            + utc_offset.to_bytes(2, "little")
        )


class CMD_UPDATE_TIME(Command):
    """Set current time on the watch."""

    cmd = 22

    def execute(
        self,
        year: int,
        month: int,
        day: int,
        hour: int,
        minute: int,
        seconds: int,
        utc_offset: int,
    ):
        return (
            self.cmd.to_bytes(1, "little")
            + (year - 2000).to_bytes(1, "little")
            + month.to_bytes(1, "little")
            + day.to_bytes(1, "little")
            + hour.to_bytes(1, "little")
            + minute.to_bytes(1, "little")
            + seconds.to_bytes(1, "little")
            + utc_offset.to_bytes(2, "little")
        )

    def process(self, data):
        return {"result": data[1]}


class CMD_RESET(Command):
    """Reset the watch."""

    cmd = 24


class CMD_ERASE_FROM_FILE(Command):
    """Erase data from a file.

    Arguments:
        file -- File to erase from FileNames(Enum)
        size -- Number of bytes to erase
    """

    cmd = 45

    def execute(self, file: FileNames, size: int):
        return (
            self.cmd.to_bytes(1, "little")
            + file.value.to_bytes(1, "little")
            + size.to_bytes(4, "little")
        )

    def process(self, data):
        return {"result": data[1]}


class CMD_GET_FILE_SIZE(Command):
    """Report file size."""

    cmd = 46

    def execute(self, file: FileNames):
        return self.cmd.to_bytes(1, "little") + file.value.to_bytes(1, "little")

    def process(self, data):
        return {
            "file": FileNames(data[1]),
            "size": struct.unpack("i", data[2:6])[0],
        }

    def str(self, data):
        return f'Selected file {data["file"].name}, file size: {data["size"]}'


class CMD_START_STREAMING_FILE(Command):
    """Start streaming from a specific file."""

    cmd = 47
    sidechannel = True

    def execute(self, file: FileNames):
        return self.cmd.to_bytes(1, "little") + file.value.to_bytes(1, "little")


class CMD_START_STREAMING_FILE_WITH_SIZE(Command):
    """Start streaming from a specific file with a defined size."""

    cmd = 48
    sidechannel = True

    def execute(self, file: FileNames, size: int):
        return (
            self.cmd.to_bytes(1, "little")
            + file.value.to_bytes(1, "little")
            + size.to_bytes(4, "little")
        )


class CMD_GET_ACTIVE_MODE(Command):
    """Get current mode from the watch."""

    cmd = 49

    def process(self, data):
        return {"mode_id": ModeID(int(data[1]))}

    def str(self, data):
        return f'Current mode {ModeID(data["mode_id"]).name}'


class CMD_GET_BATTERY_LEVEL(Command):
    """Get battery level from the watch."""

    cmd = 50

    def process(self, data):
        return {
            "level": int(data[1]),
            "voltage": struct.unpack("h", data[2:4])[0] / 1000,
            "status": CHG_STATUS(data[4]),
        }

    def str(self, data):
        return (
            f'Battery {data["level"]}% Voltage: {data["voltage"]} '
            f'status: {data["status"].name}'
        )


class CMD_GET_CURRENT_TIME(Command):
    """Get time from the watch."""

    cmd = 51

    def process(self, data):
        return {"time": datetime.fromtimestamp(struct.unpack("i", data[1:5])[0])}

    def str(self, data):
        return f'Current time {data["time"]}'


class CMD_GET_CURRENT_FILE(Command):
    """Get selected file from the watch."""

    cmd = 58

    def process(self, data):
        return {"file": FileNames(data[1])}

    def str(self, data):
        return f'Current file {data["file"].name}'


class CMD_GET_STREAMING_STATE(Command):
    """Get streaming state from the watch."""

    cmd = 59

    def process(self, data):
        return {"streaming": bool(data[1])}

    def str(self, data):
        return f'Currently {"" if data["streaming"] else "not "}streaming'


class CMD_START_STREAMING_FILE_WITH_SIZE_OFFSET(Command):
    """Start streaming from a file with size and offset."""

    cmd = 68
    sidechannel = True

    def execute(self, file: FileNames, size: int, offset: int):
        return (
            self.cmd.to_bytes(1, "little")
            + file.value.to_bytes(1, "little")
            + size.to_bytes(4, "little")
            + offset.to_bytes(4, "little")
        )


class APP_CMD_SET_PLAN(Command):
    """Set the currently active plan."""

    cmd = 149

    def execute(self, plan: PLAN, ppgfreq: PLAN_FREQUENCY, actfreq: int):
        return (
            self.cmd.to_bytes(1, "little")
            + plan.value.to_bytes(1, "little")
            + ppgfreq.value.to_bytes(1, "little")
            + actfreq.to_bytes(1, "little")
        )

    def process(self, data):
        return {"plan": data[1]}

    def str(self, data):
        return f'Set plan {data["plan"]} successfully'


class APP_CMD_GET_PLAN(Command):
    """Get the current measurement plan."""

    cmd = 150

    def process(self, data):
        return {
            "plan": PLAN(data[1]),
            "ppgfreq": PLAN_FREQUENCY(data[2]),
            "actfreq": int(data[3]),
        }

    def str(self, data):
        return (
            f'Current plan {data["plan"].name}, '
            f'PPG Frequency {data["ppgfreq"].name}, '
            f'Activity frequency {data["actfreq"]}'
        )


class APP_SET_PREV_USER_STATUS(Command):
    """Set Preventicus user status."""

    cmd = 189

    def execute(self, preventicus: int):
        return self.cmd.to_bytes(1, "little") + preventicus.to_bytes(1, "little")

    def process(self, data):
        return {"preventicus": int(data[1])}

    def str(self, data):
        return f'Set preventicus to {data["preventicus"]}'


class APP_GET_PREV_USER_STATUS(Command):
    """Get Preventicus user status."""

    cmd = 190

    def process(self, data):
        return {"preventicus": int(data[1])}

    def str(self, data):
        return f'Preventicus set to {data["preventicus"]}'


class APP_CMD_GET_VITAL_PARAM(Command):
    """Get current state of a vital parameter."""

    cmd = 193

    def execute(self, param: int):
        return self.cmd.to_bytes(1, "little") + param.to_bytes(1, "little")

    def process(self, data):
        return {
            "param": int(data[1]),
            "setting": struct.unpack("h", data[2:4])[0],
        }

    def str(self, data):
        return f'Vital param {data["param"]} setting {data["setting"]}'


class APP_CMD_START_SPECIAL_MODE(Command):
    """Set watch to special mode."""

    cmd = 216

    def execute(self, param: SPECIAL_MODES):
        return self.cmd.to_bytes(1, "little") + param.value.to_bytes(1, "little")


class APP_PING_SPECIAL_MODE(Command):
    """Ping special mode data parser."""

    cmd = 218

    def process(self, data):
        return {
            "q_green": int(data[2]),
            "q_red": int(data[3]),
            "q_ir": int(data[4]),
            "acc_x": struct.unpack("h", data[6:8])[0],
            "acc_y": struct.unpack("h", data[8:10])[0],
            "acc_z": struct.unpack("h", data[10:12])[0],
        }

    def str(self, data):
        return (
            f"QGreen {data['q_green']}, QRed {data['q_red']}, "
            f"QIR {data['q_ir']}, Acc: X {data['acc_x']}, "
            f"Y {data['acc_y']}, Z {data['acc_z']}"
        )


class APP_CMD_GET_AUTH_STATUS(Command):
    """Get authentication status from the watch."""

    cmd = 251

    def process(self, data):
        return {"authenticated": data[1] == 100}

    def str(self, data):
        return f'Authenticated: {data["authenticated"]}'


class APP_CMD_AUTHENTICATE(Command):
    """Authenticate this client with the watch.

    Mirrors the official app's SetAuthenticationRequest: a fixed
    model-id byte, a fixed auth-key byte, and 8 bytes derived from a
    client-chosen phone-id string (hex-decoded directly if it's 16 hex
    characters, otherwise the first 8 bytes of its SHA-256 hash).
    """

    cmd = 250
    AUTH_MODEL_ID_ANDROID = 0xA7
    AUTH_KEY = 22

    def execute(self, phone_id: str):
        return (
            self.cmd.to_bytes(1, "little")
            + self.AUTH_MODEL_ID_ANDROID.to_bytes(1, "little")
            + self.AUTH_KEY.to_bytes(1, "little")
            + self._phone_id_bytes(phone_id)
        )

    @staticmethod
    def _phone_id_bytes(phone_id: str) -> bytes:
        if len(phone_id) == 16:
            try:
                return bytes(int(phone_id[i : i + 2], 16) for i in range(0, 16, 2))
            except ValueError:
                pass
        return hashlib.sha256(phone_id.encode("utf-8")).digest()[:8]

    def process(self, data):
        return {"authenticated": data[1] == 100}

    def str(self, data):
        return f'Authenticated: {data["authenticated"]}'


class FW_SET_WAKE_UP(Command):
    """Periodic watch-to-device ping to keep the connection alive."""

    cmd = 125

    def process(self, data):
        return {
            "battery": int(data[2]),
            "ppg_size": struct.unpack("H", data[3:5])[0],
            "act_size": struct.unpack("H", data[5:7])[0],
            "hrv_size": struct.unpack("H", data[7:9])[0],
            "workout_size": struct.unpack("H", data[9:11])[0],
            "sleep_size": struct.unpack("H", data[11:13])[0],
            "log_size": struct.unpack("H", data[13:15])[0],
        }

    def str(self, data):
        return (
            f'Ping! Battery: {data["battery"]}% PPG {data["ppg_size"]} '
            f'Act {data["act_size"]} HRV {data["hrv_size"]} '
            f'Workout {data["workout_size"]} Sleep {data["sleep_size"]} '
            f'Log {data["log_size"]}'
        )


class CMD_UNKNOWN(BaseCommand):
    """Unknown command handler."""

    cmd = 222

    def process(self, data):
        return {i: int(data[i]) for i in range(len(data))}

    def str(self, data):
        return str(list(data.values()))


# =============================================================================
# Vendor (HCI) Commands
# =============================================================================


class VendorCommand(Command):
    """Generic vendor command base class for HCI opcodes."""

    hci_type = 0x01

    def _build_packet(self, opcode: int, params: bytes) -> bytes:
        return (
            self.hci_type.to_bytes(1, "little")
            + opcode.to_bytes(2, "little")
            + len(params).to_bytes(1, "little")
            + params
        )


class VENDOR_CMD_FD53(VendorCommand):
    """Vendor command 0xFD53 (BioZ recording)."""

    cmd = 0xFD53

    def execute(self):
        return self._build_packet(self.cmd, bytes.fromhex("00 01"))


class VENDOR_CMD_FD7D(VendorCommand):
    """Vendor command 0xFD7D (BioZ recording)."""

    cmd = 0xFD7D

    def execute(self):
        return self._build_packet(self.cmd, bytes.fromhex("00 01"))


class VENDOR_CMD_FC2D(VendorCommand):
    """Vendor command 0xFC2D (custom opcode)."""

    cmd = 0xFC2D

    def execute(self):
        return self._build_packet(self.cmd, bytes.fromhex("01"))


class VENDOR_CMD_FD57(VendorCommand):
    """Vendor command 0xFD57 (BioZ recording)."""

    cmd = 0xFD57

    def execute(self):
        return self._build_packet(self.cmd, bytes.fromhex("00 01"))


# =============================================================================
# Command Registry
# =============================================================================


commands = (
    CMD_START_STREAMING_DATA,
    CMD_SET_NORMAL_MODE,
    CMD_ERASE_FROM_FILE,
    CMD_GET_FILE_SIZE,
    CMD_RESET,
    CMD_START_STREAMING_FILE,
    APP_CMD_UPDATE_TIME,
    CMD_START_STREAMING_FILE_WITH_SIZE,
    CMD_GET_ACTIVE_MODE,
    CMD_GET_BATTERY_LEVEL,
    CMD_GET_CURRENT_TIME,
    CMD_GET_CURRENT_FILE,
    CMD_GET_STREAMING_STATE,
    CMD_START_STREAMING_FILE_WITH_SIZE_OFFSET,
    APP_CMD_SET_PLAN,
    APP_CMD_GET_PLAN,
    APP_SET_PREV_USER_STATUS,
    APP_GET_PREV_USER_STATUS,
    APP_CMD_GET_VITAL_PARAM,
    APP_CMD_START_SPECIAL_MODE,
    APP_PING_SPECIAL_MODE,
    APP_CMD_GET_AUTH_STATUS,
    APP_CMD_AUTHENTICATE,
    FW_SET_WAKE_UP,
    CMD_UNKNOWN,
)
