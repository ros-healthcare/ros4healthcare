import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from healthcare_msgs.msg import EEG, EEGInfo, DeviceInfo
import numpy as np
from pyOpenBCI import OpenBCICyton
import uuid
from collections import deque


class OpenBCINode(Node):
    def __init__(
        self,
        sample_rate=250,
        batch_size=10,
        channel_count=8,
        port="/dev/ttyUSB0",
        daisy=False,
        gain=24,  # OpenBCI Cyton default gain
    ):
        super().__init__("openbci_driver")
        self.session_id = str(uuid.uuid4())
        self.channel_count = channel_count
        self.sample_rate = sample_rate
        self.batch_size = batch_size
        self.sample_buffer = deque(maxlen=batch_size)
        self.info_published = False
        self.SCALE_UV = 4.5 / gain / (2**23 - 1)  # µV per count

        self.publisher_eeg = self.create_publisher(EEG, "eeg_raw", 10)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_info = self.create_publisher(EEGInfo, "eeg_info", latched_qos)

        self.get_logger().info(
            f"OpenBCI ROS 2 driver started. Session: {self.session_id}"
        )

        # Setup OpenBCI board
        self.board = OpenBCICyton(port=port, daisy=daisy)
        self.board.start_stream(self.callback)

        self.info_published = False

    def callback(self, sample):
        # Scale raw counts to µV (OpenBCI Cyton scale factor)

        # Store scaled channel data as plain Python floats
        self.sample_buffer.append(
            [float(v * self.SCALE_UV) for v in sample.channels_data]
        )

        if len(self.sample_buffer) >= self.batch_size:
            batch = np.array(
                self.sample_buffer, dtype=np.float32
            ).T  # (channels, batch_size)

            eeg_msg = EEG()
            eeg_msg.header.stamp = self.get_clock().now().to_msg()
            eeg_msg.header.frame_id = "openbci"
            eeg_msg.session_id = self.session_id
            eeg_msg.sample_size = self.batch_size
            eeg_msg.eeg = [float(v) for v in batch.flatten().tolist()]  # explicit cast
            eeg_msg.quality = [1.0] * (self.channel_count * self.batch_size)
            self.publisher_eeg.publish(eeg_msg)

            self.sample_buffer.clear()

            # Publish EEGInfo once
            if not self.info_published:
                info_msg = EEGInfo()
                device_msg = DeviceInfo()
                device_msg.session_id = self.session_id
                device_msg.device_name = "OpenBCI Cyton"
                device_msg.device_manufacturer = "OpenBCI"
                device_msg.device_serial_number = "UNKNOWN"
                info_msg.device_info = device_msg
                info_msg.channel_size = self.channel_count
                info_msg.units = EEGInfo.UNIT_UV
                info_msg.selected_preprocessing = []
                info_msg.montage_type = EEGInfo.MONTAGE_TYPE_UNKNOWN
                info_msg.placement_method = [
                    EEGInfo.PLACEMENT_METHOD_UNKNOWN
                ] * self.channel_count
                info_msg.electrode_sites = [
                    EEGInfo.ELECTRODE_UNKNOWN
                ] * self.channel_count
                info_msg.electrode_physical_type = [
                    EEGInfo.ELECTRODE_PHYSICAL_UNKNOWN
                ] * self.channel_count
                info_msg.signal_mode = EEGInfo.SIGNAL_MODE_SURFACE
                self.publisher_info.publish(info_msg)
                self.info_published = True


def main(args=None):
    rclpy.init(args=args)
    node = OpenBCINode(batch_size=10, channel_count=8, port="/dev/ttyUSB0", daisy=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OpenBCI driver...")
    finally:
        node.board.stop_stream()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
