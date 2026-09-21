from __future__ import annotations
import io
import time
import threading
from datetime import datetime
from logging import debug, error, info, warning

import simplepyble

from corsano_ros.commands import (
    commands,
    APP_CMD_SET_PLAN,
    FW_SET_WAKE_UP,
    CMD_UNKNOWN,
)
from corsano_ros.corsano_enums import PLAN, PLAN_FREQUENCY, check_crc
from corsano_ros.helpers import is_mac_address

# -------------------------------------------------------------------------
# BLE UUID Constants
# -------------------------------------------------------------------------
CORSANO_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
WRITE_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"
FILE_RX_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"
COMMAND_RX_CHAR = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"


class CorsanoDriver:
    """
    High-level BLE interface to a Corsano device.

    Supports both MAC and name-based scanning.
    Fully context-manager-compatible.
    """

    def __init__(
        self,
        name_or_address: str,
        adapter_name: str,
        auto_reconnect: bool = True,
        reconnect_interval: float = 5.0,
    ):
        # Adapter and connection state
        self.adapter_name = adapter_name
        self.adapter = self._get_adapter_by_name(adapter_name)
        self.peripheral = None

        # Communication and parsing attributes
        self.ping = None
        self.commands = {}
        self.stack = {}
        self.buffer = None
        self._start_tx = False
        self.hash_func = check_crc
        self.connected = False

        # Threading and reconnection management
        self._connected_event = threading.Event()
        self._reconnect_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._auto_reconnect = auto_reconnect
        self._reconnect_interval = reconnect_interval
        self._monitor_thread = None

        # Resolve address from name or direct MAC
        if is_mac_address(name_or_address):
            self.address = name_or_address
        else:
            info(f"[CorsanoDriver] Name provided instead of MAC, scanning for '{name_or_address}'...")
            self.address = self._find_device_by_name(prefix=name_or_address)

        if not self.address:
            raise RuntimeError("[CorsanoDriver] No device address provided or found.")

        # Initialize available commands
        self._init_commands()

        # Connect and optionally start monitoring
        self.connect()
        if self._auto_reconnect:
            self._start_monitor_thread()

    # ---------------------------------------------------------------------
    # Initialization Helpers
    # ---------------------------------------------------------------------
    def _get_adapter_by_name(self, adapter_name: str):
        """Return a SimplePyBLE adapter instance matching the given name."""
        for adapter in simplepyble.Adapter.get_adapters():
            if adapter.identifier() == adapter_name:
                info(f"[CorsanoDriver] Using adapter: {adapter.identifier()} ({adapter.address()})")
                return adapter
        raise RuntimeError(f"Bluetooth adapter '{adapter_name}' not found.")

    def _find_device_by_name(self, prefix: str = "287-2B", timeout_ms: int = 5000) -> str:
        """Scan for nearby BLE devices and return address of first match by name prefix."""
        info(f"[CorsanoDriver] Scanning for BLE devices (prefix='{prefix}')...")
        self.adapter.scan_for(timeout_ms)
        results = self.adapter.scan_get_results()

        for device in results:
            name = device.identifier() or device.address()
            if name.startswith(prefix):
                addr = device.address()
                info(f"[CorsanoDriver] Found matching device: {name} ({addr})")
                return addr

        info(f"[CorsanoDriver] No device found starting with '{prefix}', trying extended scan...")
        self.adapter.scan_for(timeout_ms * 2)
        results = self.adapter.scan_get_results()

        for device in results:
            name = device.identifier() or device.address()
            if name.startswith(prefix):
                addr = device.address()
                info(f"[CorsanoDriver] Found matching device: {name} ({addr})")
                return addr

        error(f"[CorsanoDriver] Device with prefix '{prefix}' not found after rescan.")
        return None

    def _init_commands(self):
        """Load available command definitions."""
        for cmd in commands:
            self.commands[cmd.cmd] = cmd()

    # ---------------------------------------------------------------------
    # Connection Handling
    # ---------------------------------------------------------------------
    def connect(self, max_retries: int = 10) -> bool:
        """Attempt to connect to the target device."""
        if not self.adapter:
            return False

        info(f"[CorsanoDriver] Connecting to {self.address}...")
        peripherals = self.adapter.get_paired_peripherals()
        retries = max_retries

        while retries > 0 and self.address not in [p.address() for p in peripherals]:
            info(f"Searching for device... ({max_retries - retries + 1}/{max_retries})")
            self.adapter.scan_for(3000)
            peripherals.extend(self.adapter.scan_get_results())
            retries -= 1

        if self.address not in [p.address() for p in peripherals]:
            error(f"[CorsanoDriver] Device {self.address} not found.")
            return False

        self.peripheral = {p.address(): p for p in peripherals}[self.address]

        attempt = 1
        while not self.connected:
            try:
                self.peripheral.connect()
                if self.peripheral.is_connected():
                    self.peripheral.notify(CORSANO_SERVICE, FILE_RX_CHAR, self._on_file_data)
                    self.peripheral.notify(CORSANO_SERVICE, COMMAND_RX_CHAR, self._on_command_data)
                    self._connected_event.set()
                    self.connected = True
                    info(
                        f"[CorsanoDriver] {datetime.now().isoformat(sep=' ', timespec='milliseconds')} "
                        f"Connected to {self.address}"
                    )
            except RuntimeError:
                info(f"[CorsanoDriver] Connection failed, retrying ({attempt + 1}/2)...")
                attempt += 1
                time.sleep(1)
        return True

    def _start_monitor_thread(self):
        """Start background thread to automatically reconnect if disconnected."""
        def monitor():
            while not self._stop_event.is_set():
                time.sleep(self._reconnect_interval)
                if self.peripheral and not self.peripheral.is_connected():
                    warning(
                        f"[CorsanoDriver] {datetime.now().isoformat(sep=' ', timespec='milliseconds')} "
                        "Device disconnected. Reconnecting..."
                    )
                    self.connected = False
                    self._attempt_reconnect()

        self._monitor_thread = threading.Thread(target=monitor, daemon=True)
        self._monitor_thread.start()

    def _attempt_reconnect(self, max_attempts: int = 5):
        """Attempt to re-establish BLE connection after disconnection.

        Does not unpair the device — an already-bonded device should be able
        to reconnect directly, and unpairing forces a fresh pairing handshake
        that requires re-confirming the pairing prompt on the host. Backs off
        between attempts instead of recursing unbounded on failure.
        """
        with self._reconnect_lock:
            self._connected_event.clear()
            for attempt in range(1, max_attempts + 1):
                try:
                    self.connect()
                    return
                except Exception as e:
                    info(
                        f"[CorsanoDriver] Reconnect attempt {attempt}/{max_attempts} failed: {e}"
                    )
                    time.sleep(min(2 ** attempt, self._reconnect_interval))
            warning(f"[CorsanoDriver] Giving up reconnecting after {max_attempts} attempts.")

    # ---------------------------------------------------------------------
    # Event Handlers
    # ---------------------------------------------------------------------
    def _on_file_data(self, data: bytes):
        """Callback invoked when file data is received."""
        if self.buffer:
            self._start_tx = True
            self.buffer.write(data)
        else:
            debug(
                f"[CorsanoDriver] {datetime.now().isoformat(sep=' ', timespec='milliseconds')} "
                f"Unexpected file-channel data with no open buffer ({len(data)} bytes): {data.hex()}"
            )

    def _on_command_data(self, data: bytes):
        """Callback invoked when command data is received."""
        if self.hash_func(data) != 0:
            debug("[CorsanoDriver] CRC check failed")
            return

        cmd_id = data[0]
        cmd = self.commands.get(cmd_id)
        if not cmd:
            debug(f"[CorsanoDriver] Unknown command: {cmd_id}")
            return

        try:
            if cmd_id == FW_SET_WAKE_UP.cmd:
                payload = cmd.process(data)
                debug(
                    f"[CorsanoDriver] {datetime.now().isoformat(sep=' ', timespec='milliseconds')} "
                    f"{cmd.str(payload)}"
                )
                if self.ping:
                    self.ping.update(cmd, payload)
            elif cmd_id == CMD_UNKNOWN.cmd and self.ping:
                self.ping.update(cmd, cmd.process(data))
            elif isinstance(self.stack.get(cmd_id), threading.Event):
                self.stack[cmd_id].set()
                self.stack[cmd_id] = cmd.process(data)
            elif hasattr(cmd, "process"):
                debug(cmd.process(data))
            else:
                debug(data)
        except Exception as e:
            import traceback
            error(f"[CorsanoDriver] Error processing command {cmd_id}: {e}")
            error(traceback.format_exc())

    # ---------------------------------------------------------------------
    # Command Execution
    # ---------------------------------------------------------------------
    def execute(self, cmd_id, *args, **kwargs):
        """Execute a command on the connected device."""
        if not self.connected:
            warning("[CorsanoDriver] Cannot execute command because the device is not connected.")
            return

        cmd = self.commands[cmd_id]
        debug(f"[CorsanoDriver] Executing command {cmd}")

        if cmd.sidechannel:
            if self.buffer:
                self.buffer.close()
            self.buffer = io.BytesIO()
            self._start_tx = False

        if not self.peripheral or not self.peripheral.is_connected():
            error("Device not connected")
            return

        self.peripheral.write_command(CORSANO_SERVICE, WRITE_CHAR, cmd.execute(*args, **kwargs))

        if hasattr(cmd, "process"):
            self.stack[cmd_id] = threading.Event()
            if not self.stack[cmd_id].wait(5.0):
                raise TimeoutError(f"Command {cmd_id} timed out")
            return self.stack[cmd_id]
        return

    def set_max_act_plan(self):
        """Set maximum activity plan configuration on device."""
        return self.execute(
            APP_CMD_SET_PLAN.cmd,
            plan=PLAN.HOSPITAL_MULTICOLOR,
            ppgfreq=PLAN_FREQUENCY["FREQ_512HZ"],
            actfreq=1,
        )

    def get_buffer(self):
        """Return and reset the internal transmission buffer."""
        if self.buffer:
            self.buffer.flush()
            self.buffer.seek(0)
            return self.buffer

    def write(self, data: bytes):
        """Write raw bytes directly to the device."""
        if not self.peripheral or not self.peripheral.is_connected():
            error("Device not connected")
            return
        self.peripheral.write_command(CORSANO_SERVICE, WRITE_CHAR, data)

    # ---------------------------------------------------------------------
    # Context Management
    # ---------------------------------------------------------------------
    def __enter__(self):
        if not self.peripheral or not self.peripheral.is_connected():
            if not self.connect():
                raise RuntimeError("Failed to connect to Corsano device")
        return self

    def __exit__(self, _exc_type, _exc_val, _exc_tb):
        self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2)
        if self.peripheral and self.peripheral.is_connected():
            self.peripheral.disconnect()
        info("[CorsanoDriver] Disconnected cleanly.")
