#!/usr/bin/env python3

import argparse
import logging
import sys
from logging import error
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int32, Float32MultiArray, Float32

from corsano_ros.corsano_driver import CorsanoDriver
from corsano_ros.helpers import load_config
from corsano_ros.retrieve_data import (
    get_last_activity_data,
    get_last_bioz_data,
    get_last_stress_data,
    get_last_accelerometer_data,
    dump_bioz_file,
)
from corsano_ros.commands import (
    VENDOR_CMD_FD53,
    VENDOR_CMD_FD7D,
    VENDOR_CMD_FC2D,
    VENDOR_CMD_FD57,
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
    clean_args = remove_ros_args(sys.argv)
    return parser.parse_args(clean_args[1:])


class CorsanoRosWrapper(Node):
    """ROS 2 wrapper around the CorsanoDriver BLE interface."""

    def __init__(self, driver: CorsanoDriver, config: dict = None):
        super().__init__("corsano_wrapper")

        self.driver = driver
        self.address = driver.address
        self.adapter_name = driver.adapter_name

        # Config-file values (if any) become the parameter defaults; an
        # explicit --ros-args -p override still takes precedence over both.
        config = config or {}
        streams_cfg = config.get("publish_streams", {})

        self.declare_parameter("publish_hr", bool(streams_cfg.get("hr", True)))
        self.publish_hr = self.get_parameter("publish_hr").value

        self.declare_parameter("publish_bioz", bool(streams_cfg.get("bioz", False)))
        self.publish_bioz = self.get_parameter("publish_bioz").value

        self.declare_parameter("publish_accel", bool(streams_cfg.get("accel", False)))
        self.publish_accel = self.get_parameter("publish_accel").value

        self.declare_parameter("publish_stress", bool(streams_cfg.get("stress", False)))
        self.publish_stress = self.get_parameter("publish_stress").value

        self.declare_parameter("publish_battery", bool(streams_cfg.get("battery", True)))
        self.publish_battery = self.get_parameter("publish_battery").value

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

        # --- Battery level polling ---
        # Runs regardless of connect_only, independent of the other streams.
        self.battery_pub = None
        self.battery_timer = None
        if self.publish_battery:
            self.battery_pub = self.create_publisher(Int32, "corsano/battery", 10)
            self.battery_timer = self.create_timer(5.0, self.request_battery_level)

        # --- Keep-alive ---
        # The watch actively disconnects (~45s after connecting) unless it
        # receives its own FW_SET_WAKE_UP ping (cmd 125) back.
        self.keep_alive_timer = self.create_timer(10.0, self.send_keep_alive)

        self.hr_pub = None
        self.rr_pub = None
        self.bioz_pub = None
        self.bioz_pub2 = None
        self.accel_pub = None
        self.stress_pub = None
        self.activity_timer = None
        self.acceleration_timer = None
        self.bioz_timer = None
        self.stress_timer = None

        if self.connect_only:
            self.get_logger().debug(
                "connect_only is set: staying connected without requesting or publishing any data."
            )
        else:
            # --- Sync the watch's clock ---
            self._set_current_time()

            # --- Authenticate ---
            self._authenticate()

            # --- Enable BioZ streaming (only needed if BioZ is published) ---
            if self.publish_bioz:
                self._enable_bioz_streaming()
            self._set_activity_plan()

            # --- ROS publishers + timers, one pair per stream ---
            if self.publish_hr:
                self.hr_pub = self.create_publisher(Int32, "corsano/hr", 10)
                self.rr_pub = self.create_publisher(Int32, "corsano/rr", 10)
                self.activity_timer = self.create_timer(0.1, self.request_activity_data)

            if self.publish_bioz:
                self.bioz_pub = self.create_publisher(
                    Float32MultiArray, "corsano/bioz", 10
                )
                self.bioz_pub2 = self.create_publisher(Float32, "corsano/bioz_val", 10)
                self.bioz_timer = self.create_timer(0.1, self.request_bioz_data)

            if self.publish_accel:
                self.accel_pub = self.create_publisher(
                    Float32MultiArray, "corsano/acceleration", 10
                )
                self.acceleration_timer = self.create_timer(
                    0.1, self.request_accelerometer_data
                )

            if self.publish_stress:
                self.stress_pub = self.create_publisher(
                    Float32MultiArray, "corsano/stress", 10
                )
                self.stress_timer = self.create_timer(0.1, self.request_stress_data)

        published_topics = [
            pub.topic_name
            for pub in (
                self.hr_pub,
                self.rr_pub,
                self.bioz_pub,
                self.bioz_pub2,
                self.accel_pub,
                self.stress_pub,
                self.battery_pub,
            )
            if pub is not None
        ]
        if published_topics:
            self.get_logger().info(f"Publishing: {', '.join(published_topics)}")
        else:
            self.get_logger().info("Publishing nothing (connect_only or no streams enabled).")

        self.get_logger().info("CorsanoWrapper node initialized and ready.")
        self.dumped_bioz_file = False

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
                self.get_logger().debug("Already authenticated.")
                return
        except Exception as e:
            self.get_logger().warning(f"Failed to get authentication status: {e}")

        self.get_logger().debug("Authenticating...")
        try:
            result = self.driver.execute(
                self.cmd_authenticate.cmd, phone_id="corsano-ros-driver"
            )
            self.get_logger().debug(f"Authenticated: {result['authenticated']}")
        except Exception as e:
            self.get_logger().warning(f"Failed to authenticate: {e}")

    def _enable_bioz_streaming(self):
        """Enable BioZ recording via vendor commands."""
        self.get_logger().debug("Enabling BioZ streaming...")
        for cmd_class in [
            VENDOR_CMD_FD53,
            VENDOR_CMD_FD7D,
            VENDOR_CMD_FC2D,
            VENDOR_CMD_FD57,
        ]:
            try:
                cmd = cmd_class()
                packet = cmd.execute()
                self.driver.write(packet)
                self.get_logger().debug(f"Sent {cmd_class.__name__} ({packet.hex()})")
            except Exception as e:
                self.get_logger().warning(f"Failed to send {cmd_class.__name__}: {e}")

    def _set_activity_plan(self):
        """Enable the device's measurement plan so it records Activity data."""
        self.get_logger().debug("Setting activity plan...")
        try:
            self.driver.set_max_act_plan()
            self.get_logger().debug("Activity plan set.")
        except Exception as e:
            self.get_logger().warning(f"Failed to set activity plan: {e}")

    # -------------------- Callbacks --------------------
    def acceleration_callback(self, acceleration: AccelerometerData):
        """Process accelerometer data and publish to ROS topic."""
        if self.accel_pub is not None and acceleration.x_values.size > 0:
            msg = Float32MultiArray()
            msg.data = [
                float(acceleration.x_values[-1]),
                float(acceleration.y_values[-1]),
                float(acceleration.z_values[-1]),
            ]
            self.accel_pub.publish(msg)
            self.get_logger().debug(
                f"Published acceleration: X={msg.data[0]:.3f}, Y={msg.data[1]:.3f}, Z={msg.data[2]:.3f}"
            )

    def activity_callback(self, activity: ActivityData):
        """Process activity data and publish HR and RR."""
        if self.hr_pub is not None and activity is not None:
            self.hr_pub.publish(Int32(data=activity.hr_filtered))
            self.rr_pub.publish(Int32(data=int(activity.rr_filtered)))
            self.get_logger().debug(
                f"Published HR={activity.hr_filtered} RR={activity.rr_filtered}"
            )

    def bioz_callback(self, bioz: BioZData):
        """Process BioZ/EDA data and publish."""
        if self.bioz_pub is not None and bioz.values.size > 0:
            msg = Float32MultiArray()
            msg.data = bioz.values.astype(float).tolist()
            self.bioz_pub.publish(msg)
            for value in bioz.values.astype(float).tolist():
                msg2 = Float32()
                msg2.data = value
                self.bioz_pub2.publish(msg2)
            self.get_logger().debug(f"Published BioZ: {len(msg.data)} samples")

    def stress_callback(self, stress: StressData):
        """Process StressData and publish to ROS topic."""
        if self.stress_pub is not None and stress is not None:
            msg = Float32MultiArray()
            # Put timestamp first, then selected stress metrics
            msg.data = [
                float(stress.timestamp_ms),
                float(stress.stress_skin),
                float(stress.stress_skin_quality),
                float(stress.pczt_min),
                float(stress.cc),
            ]
            self.stress_pub.publish(msg)
            self.get_logger().debug(
                f"Published stress: ts={msg.data[0]}, skin={msg.data[1]}, quality={msg.data[2]}, "
                f"pczt_min={msg.data[3]:.2f}, cc={msg.data[4]:.2f}"
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

    def request_battery_level(self):
        if self.driver.connected:
            try:
                battery = self.driver.execute(self.cmd_get_battery_level.cmd)
                self.battery_pub.publish(Int32(data=battery["level"]))
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

    rclpy.init(args=args)
    node = None
    try:
        with CorsanoDriver(
            name_or_address=mac_address_or_name, adapter_name=adapter_name
        ) as driver:
            node = CorsanoRosWrapper(driver, config=cfg)
            rclpy.spin(node)
    except KeyboardInterrupt:
        error("[CorsanoWrapper] User interrupt received — shutting down cleanly.")
    except rclpy.executors.ExternalShutdownException:
        error("[CorsanoWrapper] External ROS 2 shutdown requested.")
    except Exception as e:
        error(f"[CorsanoWrapper] Unhandled exception: {type(e).__name__}: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass
        except Exception as e:
            error(f"[CorsanoWrapper] Error during shutdown: {e}")


if __name__ == "__main__":
    main()
