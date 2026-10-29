#!/usr/bin/env python3

import argparse
import logging
import sys
import time
import uuid
from logging import error
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, BatteryState
from healthcare_msgs.msg import (
    DeviceInfo,
    HeartRate,
    HeartRateInfo,
    RespiratoryRate,
    RespiratoryRateInfo,
    EDA,
    EDAInfo,
    Stress,
    StressInfo,
)

from corsano_ros.corsano_enums import (
    CHG_STATUS,
    PLAN,
    PLAN_FREQUENCY,
    VitalParameterValue,
    VitalParameterWatchType,
)
from corsano_ros.corsano_driver import CorsanoDriver
from corsano_ros.helpers import load_config
from corsano_ros import retrieve_data
from corsano_ros.retrieve_data import (
    get_last_activity_data,
    get_last_bioz_data,
    get_last_stress_data,
    get_last_accelerometer_data,
    dump_bioz_file,
)
from corsano_ros.parsers.accelerometer_parser import AccelerometerData
from corsano_ros.parsers.activity_parser import ActivityData
from corsano_ros.parsers.bioz_parser import BioZData
from corsano_ros.parsers.stress_parser import StressData


def parse_args():
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(description="Corsano ROS2 wrapper node")
    parser.add_argument(
        "--config",
        type=str,
        help="Path to YAML config file containing adapter_name and mac_address_or_name.",
    )
    parser.add_argument(
        "--adapter-name",
        type=str,
        default=None,
        help="Bluetooth adapter name, e.g. hci0",
    )
    parser.add_argument(
        "--mac-address-or-name",
        type=str,
        default=None,
        help="Device MAC address or name. Defaults to '287-2B'.",
    )
    for stream, default in (
        ("hr", True),
        ("bioz", True),
        ("accel", True),
        ("stress", True),
        ("battery", True),
    ):
        parser.add_argument(
            f"--publish-{stream}",
            action=argparse.BooleanOptionalAction,
            default=None,
            help=f"Publish {stream} data (default: {default}). Overrides the config file.",
        )
    # Which measurement plan the watch runs. This matters a lot and used to be
    # hardcoded: confirmed 2026-10-01 that raw ACC only records under
    # HOSPITAL_RAW -- under the previous hardcoded HOSPITAL_MULTICOLOR the
    # ACC file never grew in any session on record, so the driver kept
    # re-publishing the same stale tail. Exposed here (and logged per run) so
    # the plan is a recorded experiment variable rather than a silent default.
    parser.add_argument(
        "--plan",
        type=str,
        default=None,
        choices=[p.name for p in PLAN],
        help="Measurement plan to set on the watch (default: HOSPITAL_MULTICOLOR). "
        "HOSPITAL_RAW is the one that records raw ACC.",
    )
    parser.add_argument(
        "--ppg-freq",
        type=str,
        default=None,
        choices=[f.name for f in PLAN_FREQUENCY],
        help="PPG sampling frequency for the plan (default: FREQ_512HZ).",
    )
    parser.add_argument(
        "--act-freq",
        type=int,
        default=None,
        help="Accelerometer frequency index for the plan (default: 1).",
    )
    parser.add_argument(
        "--test-republish-latest",
        action="store_true",
        help="TEST ONLY (wireless experiments): publish the latest BioZ and ACC "
        "record on every poll even when the watch recorded nothing new, so the "
        "topics carry a steady load regardless of skin contact. Messages may be "
        "stale -- never use this for real data collection.",
    )
    clean_args = remove_ros_args(sys.argv)
    return parser.parse_args(clean_args[1:])


class CorsanoRosWrapper(Node):
    """ROS 2 wrapper around the CorsanoDriver BLE interface."""

    def __init__(self, driver: CorsanoDriver, config: dict = None):
        super().__init__("corsano_wrapper")

        self.driver = driver
        self.address = driver.address
        self.adapter_name = driver.adapter_name
        self.session_id = str(uuid.uuid4())
        self.latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Config-file values (if any) become the parameter defaults; an
        # explicit --ros-args -p override still takes precedence over both.
        config = config or {}
        streams_cfg = config.get("publish_streams", {})

        self.declare_parameter("publish_hr", bool(streams_cfg.get("hr", True)))
        self.publish_hr = self.get_parameter("publish_hr").value

        self.declare_parameter("publish_bioz", bool(streams_cfg.get("bioz", True)))
        self.publish_bioz = self.get_parameter("publish_bioz").value

        self.declare_parameter("publish_accel", bool(streams_cfg.get("accel", True)))
        self.publish_accel = self.get_parameter("publish_accel").value

        self.declare_parameter("publish_stress", bool(streams_cfg.get("stress", True)))
        self.publish_stress = self.get_parameter("publish_stress").value

        self.declare_parameter(
            "publish_battery", bool(streams_cfg.get("battery", True))
        )
        self.publish_battery = self.get_parameter("publish_battery").value

        self.declare_parameter("plan", str(config.get("plan") or "HOSPITAL_MULTICOLOR"))
        self.plan_name = self.get_parameter("plan").value
        self.declare_parameter("ppg_freq", str(config.get("ppg_freq") or "FREQ_512HZ"))
        self.ppg_freq_name = self.get_parameter("ppg_freq").value
        self.declare_parameter("act_freq", int(config.get("act_freq") or 1))
        self.act_freq = self.get_parameter("act_freq").value

        self.declare_parameter("connect_only", False)
        self.connect_only = self.get_parameter("connect_only").value

        self.declare_parameter("debug", False)
        if self.get_parameter("debug").value:
            logging.basicConfig(level=logging.DEBUG, force=True)
            self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.get_logger().info(
            f"Using adapter: {self.adapter_name}, address: {self.address}"
        )

        # --- Map commands ---
        type_to_command = {type(v).__name__: v for v in driver.commands.values()}
        self.cmd_get_file_size = type_to_command.get("CMD_GET_FILE_SIZE")
        self.cmd_stream_file_with_size = type_to_command.get(
            "CMD_START_STREAMING_FILE_WITH_SIZE"
        )
        self.cmd_stream_file_with_size_offset = type_to_command.get(
            "CMD_START_STREAMING_FILE_WITH_SIZE_OFFSET"
        )
        self.cmd_get_battery_level = type_to_command.get("CMD_GET_BATTERY_LEVEL")
        self.cmd_wake_up = type_to_command.get("FW_SET_WAKE_UP")
        self.cmd_update_time = type_to_command.get("APP_CMD_UPDATE_TIME")
        self.cmd_get_auth_status = type_to_command.get("APP_CMD_GET_AUTH_STATUS")
        self.cmd_authenticate = type_to_command.get("APP_CMD_AUTHENTICATE")
        # Read-back commands. All three are implemented in commands.py but had
        # no call site anywhere, which is why a rejected plan or a watch in the
        # wrong mode was invisible -- see _log_device_state().
        self.cmd_set_plan = type_to_command.get("APP_CMD_SET_PLAN")
        self.cmd_set_vital_param = type_to_command.get("APP_CMD_SET_VITAL_PARAM")
        self.cmd_get_vital_param = type_to_command.get("APP_CMD_GET_VITAL_PARAM")
        self.cmd_set_normal_mode = type_to_command.get("CMD_SET_NORMAL_MODE")
        self.cmd_get_active_mode = type_to_command.get("CMD_GET_ACTIVE_MODE")
        self.cmd_get_plan = type_to_command.get("APP_CMD_GET_PLAN")
        self.cmd_get_streaming_state = type_to_command.get("CMD_GET_STREAMING_STATE")

        # --- Battery level polling ---
        # Runs regardless of connect_only, independent of the other streams.
        self.battery_pub = None
        self.battery_timer = None
        if self.publish_battery:
            self.battery_pub = self.create_publisher(
                BatteryState, "corsano/battery", 10
            )
            self.battery_timer = self.create_timer(5.0, self.request_battery_level)

        # --- Keep-alive ---
        # The watch actively disconnects (~45s after connecting) unless it
        # receives its own FW_SET_WAKE_UP ping (cmd 125) back.
        self.keep_alive_timer = self.create_timer(10.0, self.send_keep_alive)

        self.hr_pub = None
        self.hr_info_pub = None
        self.hr_info_published = False
        self.rr_pub = None
        self.rr_info_pub = None
        self.rr_info_published = False
        self.bioz_pub = None
        self.bioz_info_pub = None
        self.bioz_info_published = False
        self.accel_pub = None
        self.stress_pub = None
        self.stress_info_pub = None
        self.stress_info_published = False
        self.activity_timer = None
        self.acceleration_timer = None
        self.bioz_timer = None
        self.stress_timer = None

        if self.connect_only:
            self.get_logger().debug(
                "connect_only is set: staying connected without requesting or publishing any data."
            )
        else:
            # Run the session handshake now for the initial connection, and
            # register it to run again on every future reconnect: the watch
            # requires it (time sync + auth) on each new BLE connection, not
            # just the first one, or it silently reverts to leaving the file
            # subsystem unresponsive and disconnecting itself again after
            # ~45-50s -- exactly the original pre-auth-fix behavior.
            self._run_handshake()
            self.driver.on_reconnect = self._run_handshake

            # --- ROS publishers + timers, one pair per stream ---
            if self.publish_hr:
                self.hr_pub = self.create_publisher(HeartRate, "corsano/hr", 10)
                self.hr_info_pub = self.create_publisher(
                    HeartRateInfo, "corsano/hr_info", self.latched_qos
                )
                self.rr_pub = self.create_publisher(RespiratoryRate, "corsano/rr", 10)
                self.rr_info_pub = self.create_publisher(
                    RespiratoryRateInfo, "corsano/rr_info", self.latched_qos
                )
                self.activity_timer = self.create_timer(0.1, self.request_activity_data)

            if self.publish_bioz:
                self.bioz_pub = self.create_publisher(EDA, "corsano/bioz", 10)
                self.bioz_info_pub = self.create_publisher(
                    EDAInfo, "corsano/bioz_info", self.latched_qos
                )
                self.bioz_timer = self.create_timer(0.1, self.request_bioz_data)

            if self.publish_accel:
                self.accel_pub = self.create_publisher(Imu, "corsano/acceleration", 10)
                self.acceleration_timer = self.create_timer(
                    0.1, self.request_accelerometer_data
                )

            if self.publish_stress:
                self.stress_pub = self.create_publisher(Stress, "corsano/stress", 10)
                self.stress_info_pub = self.create_publisher(
                    StressInfo, "corsano/stress_info", self.latched_qos
                )
                self.stress_timer = self.create_timer(0.1, self.request_stress_data)

        published_topics = [
            pub.topic_name
            for pub in (
                self.hr_pub,
                self.hr_info_pub,
                self.rr_pub,
                self.rr_info_pub,
                self.bioz_pub,
                self.bioz_info_pub,
                self.accel_pub,
                self.stress_pub,
                self.stress_info_pub,
                self.battery_pub,
            )
            if pub is not None
        ]
        if published_topics:
            self.get_logger().info(f"Publishing: {', '.join(published_topics)}")
        else:
            self.get_logger().info(
                "Publishing nothing (connect_only or no streams enabled)."
            )

        self.get_logger().info("CorsanoWrapper node initialized and ready.")
        self.dumped_bioz_file = False

    def _run_handshake(self):
        """Run the session handshake the watch requires on every new
        connection: sync time, authenticate, enable BioZ (if published),
        and (re)apply the activity plan. Safe to call from the driver's
        reconnect thread as well as from __init__.
        """
        self.get_logger().debug("Running session handshake...")
        self._set_current_time()
        self._authenticate()
        self._set_activity_plan()
        if self.publish_bioz or self.publish_accel:
            self._enable_raw_recording()
        self._log_device_state()

    def _set_normal_mode(self):
        """Put the watch into MODE_NORMAL before configuring the plan.

        Confirmed 2026-10-01: this watch reports active mode 11 -- outside the
        documented ModeID set (0-9) -- with streaming False, while the plan it
        accepts reads back exactly as requested. Auth succeeds and the plan is
        applied, yet raw ACC/BioZ have never recorded in any session on record
        (corsano_bioz_file_growth_bytes is 0 in all of them). The mode is the
        one piece of state nothing in this driver ever set, so set it.
        """
        if self.cmd_set_normal_mode is None:
            self.get_logger().warning("No CMD_SET_NORMAL_MODE in the registry")
            return
        try:
            result = self.driver.execute(self.cmd_set_normal_mode.cmd)
            self.get_logger().info(f"Set normal mode; watch returned {result}")
        except Exception as e:
            self.get_logger().warning(
                f"Failed to set normal mode ({type(e).__name__}: {e})"
            )

    def _enable_raw_recording(self):
        """Switch on the watch's own recording of BioZ / ACC.

        These are per-parameter settings that persist on the watch and default
        to DISABLE, independently of the measurement plan. Confirmed
        2026-10-01: this watch reported BIOZ=[DISABLE, DISABLE] and
        ACCELEROMETER=DISABLE, and consequently raw ACC/BioZ never recorded in
        any session on record -- the driver just kept re-downloading the same
        unchanged on-watch file. Sending CONTINUOUS made ACC and PPG2 start
        growing immediately.

        This replaces four reverse-engineered vendor commands (0xFD53, 0xFD7D,
        0xFC2D, 0xFD57) that were sent in the hope of enabling BioZ; the watch
        acknowledged them but nothing ever recorded. The documented route is
        SET_VITAL_PARAM, per the Corsano BLE SDK.
        """
        if self.cmd_set_vital_param is None:
            self.get_logger().warning("No APP_CMD_SET_VITAL_PARAM in the registry")
            return
        # Mutual clobber, measured 2026-10-01: enabling BIOZ forces the plan's
        # PPG2 frequency down to FREQ_32HZ (512 -> 32 the moment the command
        # lands), and conversely re-sending SET_PLAN resets BIOZ to DISABLE
        # while leaving ACCELEROMETER alone. So BioZ recording and a high PPG2
        # rate are mutually exclusive on this watch, and the plan must be set
        # BEFORE these commands, never after -- a later plan set silently turns
        # BioZ back off.
        if self.publish_bioz and self.ppg_freq_name != "FREQ_32HZ":
            self.get_logger().warning(
                f"BioZ recording forces PPG2 to FREQ_32HZ; the requested "
                f"{self.ppg_freq_name} will not survive. Disable BioZ "
                f"(--no-publish-bioz) if the PPG2 rate matters more."
            )
        wanted = []
        if self.publish_bioz:
            # BioZ carries two values: EmoGraphy and BioZ.
            wanted.append((VitalParameterWatchType.BIOZ,
                           VitalParameterValue.CONTINUOUS,
                           VitalParameterValue.CONTINUOUS))
        if self.publish_accel:
            wanted.append((VitalParameterWatchType.ACCELEROMETER,
                           VitalParameterValue.CONTINUOUS, None))
        for param, value, value2 in wanted:
            try:
                result = self.driver.execute(
                    self.cmd_set_vital_param.cmd, param=param, value=value, value2=value2
                )
                self.get_logger().info(f"Enabled {param.name} recording: {result}")
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to enable {param.name} recording "
                    f"({type(e).__name__}: {e})"
                )
        # Read back, because the set response is only a generic ACK.
        if self.cmd_get_vital_param is not None:
            for param, _v, _v2 in wanted:
                try:
                    self.get_logger().info(
                        f"{param.name} setting now: "
                        f"{self.driver.execute(self.cmd_get_vital_param.cmd, param=param)}"
                    )
                except Exception as e:
                    self.get_logger().warning(f"{param.name} read-back failed: {e}")

    def _log_device_state(self):
        """Log what the watch says about itself once the handshake is done.

        Raw ACC/BioZ have never actually recorded on this watch:
        corsano_bioz_file_growth_bytes is 0 in every session on record (while
        the driver kept re-downloading the same static on-watch file, so the
        published numbers looked plausible). The handshake sets a plan and
        enables BioZ but never read back whether the watch accepted either,
        and never checked its mode, so there was no way to tell a rejected
        plan from a watch that simply is not logging. Report all of it at
        INFO, once per connection -- cheap, and it is the only way to tell
        these apart from the session logs afterwards.
        """
        for label, cmd in (
            ("battery", self.cmd_get_battery_level),
            ("auth status", self.cmd_get_auth_status),
            ("active mode", self.cmd_get_active_mode),
            ("plan", self.cmd_get_plan),
            ("streaming state", self.cmd_get_streaming_state),
        ):
            if cmd is None:
                self.get_logger().warning(
                    f"Device state: no {label} command in the registry"
                )
                continue
            try:
                result = self.driver.execute(cmd.cmd)
            except Exception as e:
                self.get_logger().warning(
                    f"Device state: {label} read failed ({type(e).__name__}: {e})"
                )
                continue
            self.get_logger().info(f"Device state: {label} = {result}")

    def _set_current_time(self):
        """Sync the watch's clock to the host's current local time."""
        self.get_logger().debug("Setting current time...")
        now = datetime.now().astimezone()
        utc_offset_minutes = int(now.utcoffset().total_seconds() // 60)
        try:
            self.driver.execute(
                self.cmd_update_time.cmd,
                year=now.year,
                month=now.month,
                day=now.day,
                hour=now.hour,
                minute=now.minute,
                seconds=now.second,
                utc_offset=utc_offset_minutes,
            )
            self.get_logger().debug("Current time set.")
        except Exception as e:
            self.get_logger().warning(f"Failed to set current time: {e}")

    def _authenticate(self):
        """Register this client with the watch, mirroring the official app."""
        try:
            status = self.driver.execute(self.cmd_get_auth_status.cmd)
            if status["authenticated"]:
                self.get_logger().info("Already authenticated.")
                return
        except Exception as e:
            self.get_logger().warning(f"Failed to get authentication status: {e}")

        self.get_logger().debug("Authenticating...")
        try:
            result = self.driver.execute(
                self.cmd_authenticate.cmd, phone_id="corsano-ros-driver"
            )
            self.get_logger().info(f"Authenticated: {result['authenticated']}")
        except Exception as e:
            self.get_logger().warning(f"Failed to authenticate: {e}")

    def _set_activity_plan(self):
        """Enable the device's measurement plan so it records Activity data."""
        try:
            plan = PLAN[self.plan_name]
            ppgfreq = PLAN_FREQUENCY[self.ppg_freq_name]
        except KeyError:
            self.get_logger().error(
                f"Unknown plan/ppg_freq ({self.plan_name!r}/{self.ppg_freq_name!r}); "
                f"leaving the watch's current plan alone"
            )
            return
        try:
            self.driver.execute(
                self.cmd_set_plan.cmd, plan=plan, ppgfreq=ppgfreq, actfreq=self.act_freq
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to set activity plan: {e}")
            return
        # Read the plan back rather than trusting the set response: that
        # response's second byte is a generic ACK (100), not the plan, so it
        # cannot tell us whether the watch honoured the request.
        self.get_logger().info(
            f"Plan requested: {plan.name}/{ppgfreq.name}/actfreq={self.act_freq}"
        )
        if self.cmd_get_plan is not None:
            try:
                self.get_logger().info(
                    f"Plan read back: {self.driver.execute(self.cmd_get_plan.cmd)}"
                )
            except Exception as e:
                self.get_logger().warning(f"Plan read-back failed: {e}")

    # -------------------- Device/stream info helpers --------------------
    def _make_device_info(self, device_type: str, measuring_unit: str) -> DeviceInfo:
        info = DeviceInfo()
        info.session_id = self.session_id
        info.device_manufacturer = "Corsano"
        info.device_name = "287-2B"
        info.device_type = device_type
        info.device_serial_number = self.address
        info.measuring_unit = measuring_unit
        return info

    # -------------------- Callbacks --------------------
    def acceleration_callback(self, acceleration: AccelerometerData):
        """Process accelerometer data and publish to ROS topic."""
        if self.accel_pub is not None and acceleration.x_values.size > 0:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "corsano"
            # Orientation and angular velocity aren't available from this
            # device: mark them unset per the sensor_msgs/Imu convention
            # (first covariance element = -1).
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration.x = float(acceleration.x_values[-1])
            msg.linear_acceleration.y = float(acceleration.y_values[-1])
            msg.linear_acceleration.z = float(acceleration.z_values[-1])
            self.accel_pub.publish(msg)
            self.get_logger().debug(
                f"Published acceleration: X={msg.linear_acceleration.x:.3f}, "
                f"Y={msg.linear_acceleration.y:.3f}, Z={msg.linear_acceleration.z:.3f}"
            )

    def activity_callback(self, activity: ActivityData):
        """Process activity data and publish HR and RR."""
        if self.hr_pub is not None and activity is not None:
            stamp = self.get_clock().now().to_msg()

            hr_msg = HeartRate()
            hr_msg.header.stamp = stamp
            hr_msg.session_id = self.session_id
            hr_msg.sample_size = 1
            hr_msg.heart_rate = [float(activity.hr_filtered)]
            self.hr_pub.publish(hr_msg)

            rr_msg = RespiratoryRate()
            rr_msg.header.stamp = stamp
            rr_msg.session_id = self.session_id
            rr_msg.sample_size = 1
            rr_msg.respiratory_rate = [float(activity.rr_filtered)]
            self.rr_pub.publish(rr_msg)

            if not self.hr_info_published:
                info_msg = HeartRateInfo()
                info_msg.device_info = self._make_device_info(
                    "heart rate sensor", "bpm"
                )
                info_msg.unit = HeartRateInfo.UNIT_BPM
                info_msg.sensor_placement = [HeartRateInfo.PLACEMENT_WRIST]
                info_msg.measurement_method = HeartRateInfo.METHOD_PPG
                self.hr_info_pub.publish(info_msg)
                self.hr_info_published = True

            if not self.rr_info_published:
                info_msg = RespiratoryRateInfo()
                info_msg.device_info = self._make_device_info(
                    "respiration rate sensor", "bpm"
                )
                info_msg.unit = RespiratoryRateInfo.UNIT_BPM
                info_msg.sensor_placement = [RespiratoryRateInfo.PLACEMENT_WRIST]
                info_msg.measurement_method = RespiratoryRateInfo.METHOD_PPG
                self.rr_info_pub.publish(info_msg)
                self.rr_info_published = True

            self.get_logger().debug(
                f"Published HR={activity.hr_filtered} RR={activity.rr_filtered}"
            )

    def bioz_callback(self, bioz: BioZData):
        """Process BioZ (electrodermal activity) data and publish."""
        if self.bioz_pub is not None and bioz.values.size > 0:
            values = bioz.values.astype(float).tolist()
            msg = EDA()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.session_id = self.session_id
            msg.sample_size = len(values)
            msg.eda = values
            # scl/scr/signal_quality left empty: this device doesn't split
            # tonic/phasic components or report a per-sample quality score.
            self.bioz_pub.publish(msg)

            if not self.bioz_info_published:
                info_msg = EDAInfo()
                info_msg.device_info = self._make_device_info(
                    "bio-impedance sensor", "raw"
                )
                info_msg.electrode_placement = [EDAInfo.PLACEMENT_WRIST]
                info_msg.measuring_technique = EDAInfo.TECHNIQUE_EXOSOMATIC
                info_msg.current_form = EDAInfo.CURRENT_AC
                self.bioz_info_pub.publish(info_msg)
                self.bioz_info_published = True

            self.get_logger().debug(f"Published BioZ: {len(values)} samples")

    def stress_callback(self, stress: StressData):
        """Process StressData and publish to ROS topic."""
        if self.stress_pub is not None and stress is not None:
            msg = Stress()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.session_id = self.session_id
            msg.sample_size = 1
            msg.stress_level = [float(stress.stress_skin)]
            # Corsano reports this as a raw quality code, not strictly
            # normalized to 0-1 like the field's documented convention.
            msg.quality = [float(stress.stress_skin_quality)]
            self.stress_pub.publish(msg)

            if not self.stress_info_published:
                info_msg = StressInfo()
                info_msg.device_info = self._make_device_info("stress sensor", "raw")
                info_msg.measurement_method = StressInfo.METHOD_PROPRIETARY
                info_msg.sensor_placement = [StressInfo.PLACEMENT_WRIST]
                self.stress_info_pub.publish(info_msg)
                self.stress_info_published = True

            self.get_logger().debug(
                f"Published stress: level={msg.stress_level[0]}, quality={msg.quality[0]}"
            )

    # -------------------- Request functions --------------------
    def send_keep_alive(self):
        if self.driver.connected:
            try:
                self.driver.write(self.cmd_wake_up.execute())
                self.get_logger().debug(
                    f"{datetime.now().isoformat(sep=' ', timespec='milliseconds')} "
                    "Sent keep-alive ping (FW_SET_WAKE_UP)"
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to send keep-alive: {e}")

    # Maps Corsano's charge-status enum to sensor_msgs/BatteryState's
    # POWER_SUPPLY_STATUS_* constants.
    _CHG_STATUS_TO_POWER_SUPPLY_STATUS = {
        CHG_STATUS.NOT_IN_CHARGER_DISCHARGING: BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
        CHG_STATUS.IN_CHARGER_FULLY_CHARGED: BatteryState.POWER_SUPPLY_STATUS_FULL,
        CHG_STATUS.IN_CHARGER_CHARGING: BatteryState.POWER_SUPPLY_STATUS_CHARGING,
        CHG_STATUS.NOT_IN_CHARGER_CHARGING: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
    }

    def request_battery_level(self):
        if self.driver.connected:
            try:
                battery = self.driver.execute(self.cmd_get_battery_level.cmd)
                msg = BatteryState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.voltage = float(battery["voltage"])
                msg.percentage = battery["level"] / 100.0
                msg.present = True
                msg.power_supply_status = self._CHG_STATUS_TO_POWER_SUPPLY_STATUS.get(
                    battery["status"], BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                )
                self.battery_pub.publish(msg)
                self.get_logger().debug(
                    f"Published battery: {battery['level']}% ({battery['voltage']}V, {battery['status'].name})"
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to get battery level: {e}")

    def request_accelerometer_data(self):
        if self.driver.connected:
            acc = get_last_accelerometer_data(
                self.driver,
                self.cmd_get_file_size,
                self.cmd_stream_file_with_size,
                self.cmd_stream_file_with_size_offset,
            )
            if acc is not None:
                self.acceleration_callback(acc)

    def request_activity_data(self):
        if self.driver.connected:
            activity = get_last_activity_data(
                self.driver,
                self.cmd_get_file_size,
                self.cmd_stream_file_with_size,
                self.cmd_stream_file_with_size_offset,
            )
            if activity is not None:
                self.activity_callback(activity)

    def request_bioz_data(self):
        if self.driver.connected:

            # print("Downloading BIOz file")
            # timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # filename = f"bioz_raw_2025_11_24.bin"
            # dump_bioz_file(
            #     self.driver,
            #     self.cmd_get_file_size,
            #     self.cmd_stream_file_with_size,
            #     self.cmd_stream_file_with_size_offset,
            #     filename,
            # )
            # self.dumped_bioz_file = True

            bioz = get_last_bioz_data(
                self.driver,
                self.cmd_get_file_size,
                self.cmd_stream_file_with_size,
                self.cmd_stream_file_with_size_offset,
            )
            if bioz is not None:
                self.bioz_callback(bioz)

    def request_stress_data(self):
        if self.driver.connected:
            stress = get_last_stress_data(
                self.driver,
                self.cmd_get_file_size,
                self.cmd_stream_file_with_size,
                self.cmd_stream_file_with_size_offset,
            )
            if stress is not None:
                self.stress_callback(stress)

    # -------------------- Cleanup --------------------
    def destroy_node(self):
        """Clean shutdown of node and BLE connection."""
        self.get_logger().info("Shutting down CorsanoWrapper node...")
        try:
            if (
                self.driver
                and self.driver.peripheral
                and self.driver.peripheral.is_connected()
            ):
                self.driver.peripheral.disconnect()
        except Exception as e:
            self.get_logger().warning(f"Error during disconnect: {e}")
        super().destroy_node()


def main(args=None):
    cli_args = parse_args()
    adapter_name = cli_args.adapter_name
    mac_address_or_name = cli_args.mac_address_or_name
    cfg = {}

    if cli_args.config:
        cfg = load_config(cli_args.config)
        adapter_name = adapter_name or cfg.get("adapter_name")
        mac_address_or_name = mac_address_or_name or cfg.get("mac_address_or_name")

    adapter_name = adapter_name or "hci0"
    mac_address_or_name = mac_address_or_name or "287-2B"

    # CLI --publish-<stream>/--no-publish-<stream> flags override the config file.
    streams_cfg = dict(cfg.get("publish_streams", {}))
    for stream in ("hr", "bioz", "accel", "stress", "battery"):
        cli_value = getattr(cli_args, f"publish_{stream}")
        if cli_value is not None:
            streams_cfg[stream] = cli_value
    cfg["publish_streams"] = streams_cfg

    # Same precedence for the plan: CLI over config file over the default.
    for key, cli_key in (("plan", "plan"), ("ppg_freq", "ppg_freq"),
                         ("act_freq", "act_freq")):
        cli_value = getattr(cli_args, cli_key)
        if cli_value is not None:
            cfg[key] = cli_value

    if cli_args.test_republish_latest:
        # TEST ONLY -- see retrieve_data.TEST_REPUBLISH_LATEST.
        retrieve_data.TEST_REPUBLISH_LATEST.update({"BioZ", "ACC"})
        logging.warning(
            "[CorsanoWrapper] TEST MODE (--test-republish-latest): BioZ and ACC "
            "are republished on every poll even when the watch recorded nothing "
            "new -- these messages may be stale and are for link testing only."
        )

    rclpy.init(args=args)
    try:
        # A failure anywhere in this cycle (most commonly CorsanoDriver
        # failing to find/connect to the device within its own retry
        # window, e.g. under radio interference) used to propagate all the
        # way up and exit the whole process cleanly (returncode 0) after a
        # single attempt. Retry the full connect+spin cycle instead.
        while rclpy.ok():
            node = None
            try:
                with CorsanoDriver(
                    name_or_address=mac_address_or_name, adapter_name=adapter_name
                ) as driver:
                    node = CorsanoRosWrapper(driver, config=cfg)
                    rclpy.spin(node)
            except KeyboardInterrupt:
                error(
                    "[CorsanoWrapper] User interrupt received — shutting down cleanly."
                )
                break
            except rclpy.executors.ExternalShutdownException:
                error("[CorsanoWrapper] External ROS 2 shutdown requested.")
                break
            except Exception as e:
                error(
                    f"[CorsanoWrapper] Unhandled exception: {type(e).__name__}: {e}. "
                    "Retrying..."
                )
                time.sleep(5)
            finally:
                if node is not None:
                    node.destroy_node()
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass
        except Exception as e:
            error(f"[CorsanoWrapper] Error during shutdown: {e}")


if __name__ == "__main__":
    main()
