from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
import asyncio
import logging as log
import time
import uuid
import numpy as np
import math
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from healthcare_msgs.msg import HeartRate, HeartRateInfo, ECG, ECGInfo, DeviceInfo
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState


class PolarH10:
    HEART_RATE_SERVICE_UUID = "0000180d-0000-1000-8000-00805f9b34fb"
    HEART_RATE_MEASUREMENT_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
    PMD_SERVICE_UUID = "fb005c80-02e7-f387-1cad-8acd2d8df0c8"
    PMD_CHAR1_UUID = "fb005c81-02e7-f387-1cad-8acd2d8df0c8"
    PMD_CHAR2_UUID = "fb005c82-02e7-f387-1cad-8acd2d8df0c8"
    # Standard Bluetooth SIG Battery Service / Battery Level characteristic.
    BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"
    BATTERY_POLL_INTERVAL_SEC = 60.0
    # Wait this long after connecting before the first battery read, so it
    # doesn't add extra GATT/service-discovery traffic right in the fragile
    # connection-setup window.
    BATTERY_FIRST_READ_DELAY_SEC = 10.0

    # A connection that survives at least this long counts as "stable" and
    # resets the reconnect backoff below.
    RECONNECT_STABLE_THRESHOLD_SEC = 10.0
    RECONNECT_MAX_BACKOFF_SEC = 30.0
    # Outer bound on one bleak connect() call (its own timeout is 20s).
    CONNECT_HARD_DEADLINE_SEC = 35.0
    # Retry delay after a transient host-side connect failure (see
    # _is_transient_connect_glitch) -- these don't need exponential backoff.
    FAST_RETRY_DELAY_SEC = 1.0

    ECG_WRITE = bytearray([0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00])
    ACC_WRITE = bytearray(
        [
            0x02,
            0x02,
            0x00,
            0x01,
            0xC8,
            0x00,
            0x01,
            0x01,
            0x10,
            0x00,
            0x02,
            0x01,
            0x08,
            0x00,
        ]
    )

    ACC_SAMPLING_FREQ = 200
    ECG_SAMPLING_FREQ = 130
    MAX_DATA_AGE_SEC = 1.0

    def __init__(
        self,
        node,
        bleak_device,
        bt_adapter,
        publish_acceleration,
        publish_hr,
        publish_ecg,
        publish_battery=True,
        ecg_per_frame=False,
    ):
        self.bleak_device = bleak_device
        self.bt_adapter = bt_adapter
        self.node = node
        self.publish_acceleration = publish_acceleration
        self.publish_hr = publish_hr
        self.publish_ecg = publish_ecg
        self.publish_battery = publish_battery
        # One ECG message per BLE frame (sample_size = samples in the frame)
        # instead of one per sample: a frame's ~73 samples then cost one
        # publish, so frames arriving in a burst no longer queue behind
        # thousands of single-sample publishes (live-path tail).
        self.ecg_per_frame = ecg_per_frame
        self.combined_streaming_active = False
        self.hr_streaming_active = False
        self.session_id = str(uuid.uuid4())
        self.hr_info_published = False
        self.ecg_info_published = False
        self.bleak_client = None
        self._battery_task = None
        self._last_connected_at = None
        self._reconnect_backoff = 0.0
        # Set once an intentional shutdown starts, so the disconnect callback
        # that BlueZ fires during teardown doesn't kick off a reconnect (and a
        # 20s connect timeout) while the node is trying to exit.
        self._closing = False
        # Strong reference to the reconnect task. A bare asyncio.create_task()
        # result is only weakly referenced by the loop, so it can be garbage
        # collected mid-flight -- which would silently end the reconnect chain,
        # the same way an uncaught exception used to.
        self._reconnect_task = None

        self.acc_stream_values = []
        self.acc_stream_times = []
        self.ecg_stream_values = []
        self.ecg_stream_times = []
        self.ibi_stream_values = []
        self.ibi_stream_times = []

        # Guards against handle_disconnect() spawning a competing connect()
        # task while one (the initial connect, or an earlier reconnect) is
        # already retrying -- confirmed on real hardware to otherwise cause
        # concurrent connect attempts that fight each other with "Operation
        # already in progress" / "br-connection-canceled" from BlueZ.
        self._connect_lock = asyncio.Lock()

        if publish_acceleration:
            self.pub_acc = self.node.create_publisher(Vector3Stamped, "polar_acc", 10)
        if publish_hr:
            self.pub_hr = self.node.create_publisher(HeartRate, "polar_hr", 10)
            latched_qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            self.pub_hr_info = self.node.create_publisher(
                HeartRateInfo, "polar_hr_info", latched_qos
            )
        if publish_ecg:
            self.pub_ecg = self.node.create_publisher(ECG, "polar_ecg", 10)
            ecg_latched_qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            self.pub_ecg_info = self.node.create_publisher(
                ECGInfo, "polar_ecg_info", ecg_latched_qos
            )
        if publish_battery:
            self.pub_battery = self.node.create_publisher(
                BatteryState, "polar_battery", 10
            )

    async def connect(self, max_retries: int = 10):
        """Connect (or reconnect), retrying with backoff. Returns True once the
        link is up and the requested streams are running.

        Returns a bool rather than None because every caller needs to be able
        to tell "connected" from "gave up": _reconnect_with_backoff() treats a
        falsy result as a failure and keeps trying. Previously this returned
        early -- indistinguishable from success -- whenever another attempt
        held the lock, which made the reconnect chain stop for the rest of the
        node's life the first time the two raced.
        """
        if self._connect_lock.locked():
            log.debug(
                "[PolarH10::connect] A connect attempt is already in progress; "
                "waiting for it rather than starting a duplicate."
            )
            async with self._connect_lock:
                pass
            return bool(self.bleak_client and self.bleak_client.is_connected)
        is_reconnect = self._last_connected_at is not None
        async with self._connect_lock:
            slow_failures = 0
            for attempt in range(1, max_retries + 1):
                if self._closing:
                    return False
                try:
                    self.bleak_client = BleakClient(
                        self.bleak_device,
                        adapter=self.bt_adapter,
                        disconnected_callback=self.handle_disconnect,
                    )
                    log.info(f"[PolarH10::connect] Attempt {attempt}/{max_retries}")
                    # Hard outer deadline. When its own 20s timeout fires, bleak
                    # cancels the pending connect with a D-Bus Disconnect call that
                    # has no timeout; if bluetoothd never answers it (confirmed
                    # 2026-09-28: a failed MGMT Disconnect, status 0x0e, leaves
                    # BlueZ holding a stale "connected" device), this await never
                    # returned and the node went silent with the process alive.
                    t_attempt = time.monotonic()
                    try:
                        await asyncio.wait_for(
                            self.bleak_client.connect(timeout=20.0),
                            self.CONNECT_HARD_DEADLINE_SEC,
                        )
                    except asyncio.TimeoutError:
                        # bleak's own 20s timeout raises TimeoutError too; only
                        # blame BlueZ when the outer deadline was what fired.
                        hung = (time.monotonic() - t_attempt
                                >= self.CONNECT_HARD_DEADLINE_SEC - 1.0)
                        if hung and not self._closing:
                            log.error(
                                "[PolarH10::connect] bleak did not return within "
                                f"{self.CONNECT_HARD_DEADLINE_SEC:.0f}s -- BlueZ is likely "
                                "holding a stale connection (see journalctl -u bluetooth "
                                "for 'Failed to disconnect device'); an adapter power "
                                "cycle or bluetoothd restart clears it."
                            )
                        raise
                    print("Connected!")
                    self._last_connected_at = time.monotonic()
                    # A reconnect that comes back without its notification
                    # subscriptions is worse than a visible failure: the node
                    # looks healthy and publishes nothing. Treat it as a failed
                    # attempt and try again instead of settling for a silent link.
                    if is_reconnect and not await self._resume_streams():
                        await self._drop_client()
                        raise BleakError("streams did not resume after reconnect")
                    if self.publish_battery:
                        self._start_battery_polling()
                    return True
                except (asyncio.TimeoutError, BleakError) as e:
                    glitch = self._is_transient_connect_glitch(e)
                    if glitch:
                        delay = self.FAST_RETRY_DELAY_SEC
                    else:
                        slow_failures += 1
                        delay = min(2 ** slow_failures, 30)
                    log.error(
                        f"[PolarH10::connect] Connection attempt {attempt}/{max_retries} "
                        f"failed: {type(e).__name__}: {str(e) or 'no details'}. "
                        f"Retrying in {delay:.0f}s..."
                    )
                    if self._closing:
                        return False
                    if attempt == max_retries:
                        break
                    await asyncio.sleep(delay)
                    # Always re-scan before the next attempt: BlueZ drops a
                    # device it hasn't seen in a scan for 30s (TemporaryTimeout),
                    # so reusing the startup BLEDevice across backoff sleeps
                    # left later attempts pointing at a removed or stale object
                    # (session 20260930_094934: "Device ... was not found" and
                    # 20s timeouts until the outer loop re-scanned).
                    if not self._closing:
                        await self._rediscover()
            raise BleakError(
                f"[PolarH10::connect] Giving up after {max_retries} attempts."
            )

    @staticmethod
    def _is_transient_connect_glitch(exc):
        """True for connect failures that the very next attempt usually
        clears, as opposed to the strap being unreachable (timeouts).

        "failed to discover services, device disconnected": a bleak 0.21
        bug, not a link failure. While connect() waits for services, bleak
        treats the removal of ANY device on the adapter (BlueZ expiring an
        unrelated nearby advertiser from its cache) as this device being
        removed (bluezdbus/manager.py: `obj_path.startswith(adapter_path)`),
        then calls Disconnect itself. Confirmed 2026-09-30 with a root
        dbus-monitor trace: 4/4 failures had bleak's Disconnect call 2-3ms
        after another device's InterfacesRemoved; ~15-30% of connects in a
        busy RF room, and the next attempt succeeded every time. "not
        found": the BlueZ device object itself was removed; the re-scan
        before the next attempt recreates it."""
        if isinstance(exc, asyncio.TimeoutError):
            return False
        text = str(exc)
        return "failed to discover services" in text or "was not found" in text

    async def _drop_client(self):
        """Tear down a half-open client so the next attempt starts clean."""
        try:
            if self.bleak_client and self.bleak_client.is_connected:
                await self.bleak_client.disconnect()
        except Exception as e:
            log.debug(f"[PolarH10] Ignoring error while dropping client: {e}")

    async def _rediscover(self, timeout: float = 10.0):
        """Re-scan for the strap and refresh the cached BLEDevice.

        Captured under forced-channel interference: after a supervision-timeout
        drop the H10 was still radiating strongly (RSSI -42..-53 dBm) but only
        as ADV_NONCONN_IND, i.e. non-connectable, so BlueZ's accept-list
        connect path had nothing to act on and every attempt burned its full
        20s timeout. Re-scanning between connect cycles both refreshes the
        device object BlueZ may have dropped from its cache and tells us
        whether the strap is advertising connectably again yet.
        """
        address = getattr(self.bleak_device, "address", None)
        if not address:
            return False
        try:
            found = await BleakScanner.find_device_by_address(
                address, timeout=timeout, adapter=self.bt_adapter
            )
        except Exception as e:
            log.warning(f"[PolarH10] Re-scan failed: {e}")
            return False
        if found is None:
            log.warning(
                f"[PolarH10] {address} not seen in a {timeout:.0f}s re-scan; "
                "it may be advertising non-connectably."
            )
            return False
        self.bleak_device = found
        log.info(f"[PolarH10] Re-discovered {address}.")
        return True

    def handle_disconnect(self, client):
        if self._closing:
            log.info("[PolarH10] Disconnected during shutdown; not reconnecting.")
            return
        if self._reconnect_task is not None and not self._reconnect_task.done():
            log.debug("[PolarH10] Reconnect already in progress.")
            return
        log.warning("[PolarH10] Disconnected. Reconnecting...")
        self._reconnect_task = asyncio.create_task(self._reconnect_with_backoff())

    async def _reconnect_with_backoff(self):
        """Reconnect after a disconnect, backing off if connections keep
        dying quickly, and never giving up while the node is alive.

        Confirmed on real hardware under RF interference: reconnecting
        instantly every time produces a storm of connect/disconnect cycles a
        few hundred ms apart (HCI disconnect reason 0x3E, "Connection Failed
        to be Established"), each a fresh connection that dies again almost
        immediately -- hammering the radio instead of recovering. Back off
        (growing the delay while connections keep failing fast) to give the
        link room to actually re-establish, and reset to an instant retry
        once a connection has held for a while.

        Also confirmed on real hardware: connect()'s own bounded retry loop
        can still fail to recover -- one capture showed a single failed
        attempt (right at connect()'s 20s per-attempt timeout) followed by
        total silence for the rest of a multi-minute test, no further scan
        or connect activity at all. connect() only catches
        (asyncio.TimeoutError, BleakError), but bleak's internal timeout
        handling relies on converting a cancellation into TimeoutError --
        if that conversion doesn't happen cleanly (a known rough edge in
        async D-Bus stacks under real RF interference), a bare
        asyncio.CancelledError escapes uncaught and silently kills this
        whole reconnect chain, since it's a fire-and-forget task. Loop here
        and catch broadly so no single failure, of any kind, permanently
        ends reconnection -- bounded only by the node's own lifetime so it
        can't block a real shutdown.
        """
        while rclpy.ok():
            now = time.monotonic()
            if (
                self._last_connected_at is not None
                and (now - self._last_connected_at) < self.RECONNECT_STABLE_THRESHOLD_SEC
            ):
                self._reconnect_backoff = min(
                    max(self._reconnect_backoff * 2, 2.0), self.RECONNECT_MAX_BACKOFF_SEC
                )
            else:
                self._reconnect_backoff = 0.0

            if self._reconnect_backoff:
                log.warning(
                    f"[PolarH10] Connection dropped quickly; backing off "
                    f"{self._reconnect_backoff:.1f}s before reconnecting..."
                )
                await asyncio.sleep(self._reconnect_backoff)
            if self._closing:
                return
            try:
                if await self.connect():
                    return
                log.warning("[PolarH10] Still not connected; retrying...")
            except asyncio.CancelledError:
                # Catching BaseException below is deliberate, but it must not
                # swallow a cancellation aimed at this task, or the chain
                # becomes unstoppable and shutdown hangs on it. Task.cancelling()
                # is non-zero only for a real Task.cancel(), which is how we
                # tell that apart from a stray CancelledError leaking out of
                # bleak's timeout handling -- the case this loop exists to
                # survive.
                task = asyncio.current_task()
                if self._closing or (task is not None and task.cancelling()):
                    raise
                log.error(
                    "[PolarH10] Stray CancelledError during reconnect; retrying..."
                )
            except BaseException as e:
                log.error(
                    f"[PolarH10] Reconnect attempt failed ({type(e).__name__}: "
                    f"{e}). Retrying..."
                )
            # A whole connect() cycle failed. Re-scan before the next one: the
            # cached BLEDevice may be stale, and if the strap is only
            # advertising non-connectably there is nothing to connect to yet,
            # so this also stops us from blindly burning 20s timeouts.
            if not self._closing:
                await self._rediscover()

    async def _resume_streams(self):
        """Re-subscribe to whichever notification streams were requested.

        A fresh BLE connection after any reconnect starts with none of the
        previous connection's GATT subscriptions -- the strap isn't bonded,
        so it doesn't remember them across a link-layer disconnect. Without
        this, the connection recovers but no HR/ECG/ACC data ever resumes,
        exactly the same class of bug fixed for the Corsano watch's
        session handshake not being re-run on reconnect.

        Returns True only if every requested stream came back, so connect()
        can retry instead of leaving a connected-but-silent link in place.
        """
        try:
            if self.publish_acceleration or self.publish_ecg:
                await self.start_combined_stream()
            if self.publish_hr:
                await self.start_hr_stream()
        except Exception as e:
            log.warning(f"[PolarH10] Failed to resume streams after reconnect: {e}")
            return False
        return True

    async def disconnect(self):
        # Mark shutdown first so the disconnect callback BlueZ fires below
        # doesn't start a fresh reconnect while we are tearing down.
        self._closing = True
        if self._reconnect_task is not None:
            self._reconnect_task.cancel()
            self._reconnect_task = None
        if self._battery_task is not None:
            self._battery_task.cancel()
            self._battery_task = None
        if self.bleak_client and self.bleak_client.is_connected:
            await self.bleak_client.disconnect()
            print("Disconnected.")

    def _start_battery_polling(self):
        """(Re)start the periodic battery poll. Called from connect() so it
        resumes automatically after a reconnect, not just the first
        connection."""
        if self._battery_task is not None and not self._battery_task.done():
            self._battery_task.cancel()
        self._battery_task = asyncio.create_task(self._battery_loop())

    async def _battery_loop(self):
        await asyncio.sleep(self.BATTERY_FIRST_READ_DELAY_SEC)
        while True:
            await self.read_battery_level()
            await asyncio.sleep(self.BATTERY_POLL_INTERVAL_SEC)

    async def read_battery_level(self):
        try:
            data = await self.bleak_client.read_gatt_char(self.BATTERY_LEVEL_UUID)
            level = data[0]
            msg = BatteryState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.percentage = level / 100.0
            msg.present = True
            self.pub_battery.publish(msg)
            log.debug(f"[PolarH10] Published battery: {level}%")
        except Exception as e:
            log.warning(f"[PolarH10] Failed to read battery level: {e}")

    async def start_combined_stream(self):
        await self.bleak_client.write_gatt_char(
            self.PMD_CHAR1_UUID, self.ECG_WRITE, response=True
        )
        await self.bleak_client.write_gatt_char(
            self.PMD_CHAR1_UUID, self.ACC_WRITE, response=True
        )
        await self.bleak_client.start_notify(self.PMD_CHAR2_UUID, self.pmd_data_conv)
        self.combined_streaming_active = True
        print("Streaming ECG and ACC...")

    async def stop_combined_stream(self):
        if (
            self.bleak_client
            and self.bleak_client.is_connected
            and self.combined_streaming_active
        ):
            try:
                await self.bleak_client.stop_notify(self.PMD_CHAR2_UUID)
                print("Stopped streaming ECG and ACC.")
            except Exception as e:
                log.warning(f"[PolarH10] Failed to stop notify: {e}")
            self.combined_streaming_active = False

    async def start_hr_stream(self):
        await self.bleak_client.start_notify(
            self.HEART_RATE_MEASUREMENT_UUID, self.hr_data_conv
        )
        self.hr_streaming_active = True
        print("Started HR stream.")

    async def stop_hr_stream(self):
        if (
            self.bleak_client
            and self.bleak_client.is_connected
            and self.hr_streaming_active
        ):
            try:
                await self.bleak_client.stop_notify(self.HEART_RATE_MEASUREMENT_UUID)
                print("Stopped HR stream.")
            except Exception as e:
                log.warning(f"[PolarH10] Failed to stop HR notify: {e}")
            self.hr_streaming_active = False

    def pmd_data_conv(self, sender, data):
        if data[0] == 0x00:
            self.ecg_data_conv(sender, data)
        elif data[0] == 0x02:
            self.acc_data_conv(sender, data)

    def hr_data_conv(self, sender, data):
        flags = data[0]
        hr_format = (flags & 0x01) == 0
        rr_present = (flags >> 4) & 1

        index = 2 if hr_format else 3
        hr = data[1] if hr_format else (data[2] << 8) | data[1]

        # hr == 0 is what the strap sends without skin contact -- not a
        # measurement. A valid HR is still worth publishing when a frame
        # happens to carry no RR interval.
        if self.publish_hr and hr > 0:
            hr_msg = HeartRate()
            hr_msg.header.stamp = self.node.get_clock().now().to_msg()
            hr_msg.session_id = self.session_id
            hr_msg.sample_size = 1
            hr_msg.heart_rate = [float(hr)]
            hr_msg.quality = [1.0]
            self.pub_hr.publish(hr_msg)

            if not self.hr_info_published:
                info_msg = HeartRateInfo()
                device_msg = DeviceInfo()
                device_msg.session_id = self.session_id
                device_msg.device_manufacturer = "Polar"
                device_msg.device_name = "Polar H10"
                device_msg.device_type = "heart rate sensor"
                device_msg.device_serial_number = self.bleak_device.address
                device_msg.measuring_unit = "bpm"
                info_msg.device_info = device_msg
                info_msg.unit = HeartRateInfo.UNIT_BPM
                info_msg.sensor_placement = [HeartRateInfo.PLACEMENT_CHEST]
                info_msg.measurement_method = HeartRateInfo.METHOD_ECG
                self.pub_hr_info.publish(info_msg)
                self.hr_info_published = True

        if not rr_present:
            return
        if (flags >> 3) & 1:  # Skip energy exp
            index += 2
        for i in range(index, len(data), 2):
            ibi = (data[i + 1] << 8) | data[i]
            ibi_ms = np.ceil(ibi / 1024 * 1000)
            self.ibi_stream_values.append(ibi_ms)
            self.ibi_stream_times.append(time.time_ns() / 1e9)

    def acc_data_conv(self, sender, data):
        if data[0] != 0x02:
            return
        timestamp = self.convert_to_unsigned_long(data, 1, 8) / 1e9
        frame_type = data[9]
        resolution = (frame_type + 1) * 8
        time_step = 1.0 / self.ACC_SAMPLING_FREQ
        step = math.ceil(resolution / 8.0)
        samples = data[10:]
        offset = 0
        n_samples = len(samples) // (3 * step)
        sample_timestamp = timestamp - (n_samples - 1) * time_step

        while offset + 3 * step <= len(samples):
            x = self.convert_array_to_signed_int(samples, offset, step)
            y = self.convert_array_to_signed_int(samples, offset + step, step)
            z = self.convert_array_to_signed_int(samples, offset + 2 * step, step)
            offset += 3 * step
            self.acc_stream_values.append([x, y, z])
            self.acc_stream_times.append(sample_timestamp)
            if self.publish_acceleration:
                msg = Vector3Stamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.vector.x = x / 1000
                msg.vector.y = y / 1000
                msg.vector.z = z / 1000
                self.pub_acc.publish(msg)
            sample_timestamp += time_step

    def ecg_data_conv(self, sender, data):
        timestamp = self.convert_to_unsigned_long(data, 1, 8) / 1e9
        time_step = 1.0 / self.ECG_SAMPLING_FREQ
        step = 3
        samples = data[10:]
        sample_timestamp = timestamp - ((len(samples) // step) - 1) * time_step
        offset = 0

        if self.ecg_per_frame:
            values = []
            while offset + step <= len(samples):
                ecg_val = self.convert_array_to_signed_int(samples, offset, step)
                self.ecg_stream_values.append(ecg_val)
                self.ecg_stream_times.append(sample_timestamp)
                values.append(ecg_val / 1000.0)
                offset += step
                sample_timestamp += time_step
            if self.publish_ecg and values:
                ecg_msg = ECG()
                ecg_msg.header.stamp = self.node.get_clock().now().to_msg()
                ecg_msg.session_id = self.session_id
                ecg_msg.channel_size = 1
                ecg_msg.sample_size = len(values)
                ecg_msg.ecg = values
                ecg_msg.signal_quality = [1.0] * len(values)
                self.pub_ecg.publish(ecg_msg)
                self._publish_ecg_info_once()
            return

        while offset + step <= len(samples):
            ecg_val = self.convert_array_to_signed_int(samples, offset, step)
            self.ecg_stream_values.append(ecg_val)
            self.ecg_stream_times.append(sample_timestamp)
            if self.publish_ecg:
                ecg_msg = ECG()
                ecg_msg.header.stamp = self.node.get_clock().now().to_msg()
                ecg_msg.session_id = self.session_id
                ecg_msg.channel_size = 1
                ecg_msg.sample_size = 1
                ecg_msg.ecg = [ecg_val / 1000.0]
                ecg_msg.signal_quality = [1.0]
                self.pub_ecg.publish(ecg_msg)

                self._publish_ecg_info_once()
            offset += step
            sample_timestamp += time_step

    def _publish_ecg_info_once(self):
        if self.ecg_info_published:
            return
        info_msg = ECGInfo()
        device_msg = DeviceInfo()
        device_msg.session_id = self.session_id
        device_msg.device_manufacturer = "Polar"
        device_msg.device_name = "Polar H10"
        device_msg.device_type = "ECG sensor"
        device_msg.device_serial_number = self.bleak_device.address
        device_msg.measuring_unit = "mV"
        device_msg.acquisition_rate_hz = self.ECG_SAMPLING_FREQ
        info_msg.device_info = device_msg
        info_msg.units = ECGInfo.UNIT_MV
        info_msg.electrode_placement = [ECGInfo.PLACEMENT_CHEST]
        info_msg.lead_type = [ECGInfo.LEAD_UNKNOWN]
        self.pub_ecg_info.publish(info_msg)
        self.ecg_info_published = True

    @staticmethod
    def convert_array_to_signed_int(data, offset, length):
        return int.from_bytes(
            data[offset : offset + length], byteorder="little", signed=True
        )

    @staticmethod
    def convert_to_unsigned_long(data, offset, length):
        return int.from_bytes(
            data[offset : offset + length], byteorder="little", signed=False
        )

    def get_acc_data(self):
        times = np.array(self.acc_stream_times)
        if len(times) == 0:
            return None
        return {"times": times - times[0], "values": np.array(self.acc_stream_values)}

    def get_ibi_data(self):
        if not self.ibi_stream_times:
            return None
        times = np.array(self.ibi_stream_times)
        return {"times": times - times[0], "values": np.array(self.ibi_stream_values)}
