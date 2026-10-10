#!/usr/bin/env python3
import os
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from healthcare_msgs.msg import EEG, EEGInfo, FilterBase, NotchFilter, DeviceInfo

from neurosity import NeurositySDK

load_dotenv()

REQUIRED_ENV_VARS = ["NEUROSITY_DEVICE_ID", "NEUROSITY_EMAIL", "NEUROSITY_PASSWORD"]
missing = [var for var in REQUIRED_ENV_VARS if not os.getenv(var)]
if missing:
    raise EnvironmentError(
        f"Missing required environment variables: {', '.join(missing)}"
    )


class NeurosityEEGDriver(Node):
    def __init__(self):
        super().__init__("neurosity_eeg_driver")
        self.eeg_pub = self.create_publisher(EEG, "neurosity/eeg", 10)
        self.eeg_info_pub = self.create_publisher(EEGInfo, "neurosity/eeg_info", 1)
        self.info_published = False

        self.sdk = NeurositySDK({"device_id": os.getenv("NEUROSITY_DEVICE_ID")})
        self.sdk.login(
            {
                "email": os.getenv("NEUROSITY_EMAIL"),
                "password": os.getenv("NEUROSITY_PASSWORD"),
            }
        )

        self.get_logger().info("Subscribed to Neurosity EEG stream")
        self.subscribe = self.sdk.brainwaves_raw(self.brainwave_callback)
        # Use .brainwaves_raw_unfiltered() if you want raw unfiltered

    def unsubscribe(self):
        self.get_logger().info("Unsubscribing from Neurosity EEG stream...")
        if self.subscribe:
            self.subscribe()
            self.get_logger().info("Unsubscribed from Neurosity EEG stream")

    def brainwave_callback(self, data):
        info = data["info"]
        channels = data["data"]  # list of lists of floats

        if not channels:
            return

        # Timestamp from SDK startTime (milliseconds → ROS time)
        start_time_ms = info["startTime"]
        start_time_sec = start_time_ms / 1000.0
        ros_time = rclpy.time.Time(
            seconds=int(start_time_sec), nanoseconds=int((start_time_sec % 1) * 1e9)
        )

        # EEG message
        eeg_msg = EEG()
        eeg_msg.header.stamp = ros_time.to_msg()
        eeg_msg.header.frame_id = "neurosity"
        eeg_msg.session_id = info.get("session_id", "")
        eeg_msg.sample_size = len(channels[0])

        # Flatten [channel][sample] → 1D
        flattened = []
        for ch in channels:
            flattened.extend(ch)
        eeg_msg.eeg = [float(v) for v in flattened]

        self.eeg_pub.publish(eeg_msg)

        # EEGInfo (published once)
        if not self.info_published:
            sampling_rate = info["samplingRate"]
            channel_names = info["channelNames"]
            try:
                notch_freq = info["notchFrequency"]
                notch_hz = float(
                    "".join(c for c in notch_freq if c.isdigit() or c == ".")
                )
            except (ValueError, TypeError, KeyError):
                self.get_logger().warning(
                    f"Did not find notch frequency in info: {info}"
                )
                notch_freq = None

            # DeviceInfo
            device_info = DeviceInfo()
            device_info.header.stamp = self.get_clock().now().to_msg()
            device_info.session_id = info.get("session_id", "")
            device_info.device_manufacturer = "Neurosity"
            device_info.device_name = os.getenv("NEUROSITY_DEVICE_ID")
            device_info.device_type = "EEG Headset"
            device_info.measuring_unit = "uV"
            device_info.acquisition_rate_hz = sampling_rate
            device_info.gain = 1.0
            device_info.window_size_samples = len(channels[0])

            # EEGInfo
            info_msg = EEGInfo()
            info_msg.device_info = device_info
            info_msg.channel_size = len(channels)
            info_msg.units = EEGInfo.UNIT_UV

            info_msg.montage_type = EEGInfo.MONTAGE_TYPE_REFERENTIAL
            info_msg.electrode_sites = [
                getattr(EEGInfo, f"ELECTRODE_{name.upper()}", EEGInfo.ELECTRODE_CUSTOM)
                for name in channel_names
            ]
            info_msg.electrode_physical_type = [EEGInfo.ELECTRODE_PHYSICAL_DRY] * len(
                channels
            )
            info_msg.placement_method = [EEGInfo.PLACEMENT_METHOD_1020] * len(channels)
            info_msg.signal_mode = EEGInfo.SIGNAL_MODE_SURFACE
            # NotchFilter
            if notch_freq:
                info_msg.selected_preprocessing = [
                    EEGInfo.EEG_PREPROC_NOTCH,
                ]
                notch_filter = NotchFilter()
                notch_filter.header.stamp = self.get_clock().now().to_msg()
                notch_filter.center_frequency_hz = notch_hz

                # FilterBase
                filters = FilterBase()
                filters.filters = [FilterBase.TYPE_NOTCH]
                filters.notch_filter = notch_filter
                info_msg.filters = filters

            self.eeg_info_pub.publish(info_msg)
            self.info_published = True


def main(args=None):
    rclpy.init(args=args)
    node = NeurosityEEGDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
        node.unsubscribe()
    finally:
        node.unsubscribe()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
