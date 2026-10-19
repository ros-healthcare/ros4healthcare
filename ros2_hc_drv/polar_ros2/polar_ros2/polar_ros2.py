from bleak import BleakScanner
import argparse
import asyncio
import logging as log
import sys
import time
from .submodules.PolarH10 import PolarH10
from .submodules.BreathingAnalyser import BreathingAnalyser
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Vector3Stamped


def parse_args():
    """Parse CLI arguments (mirrors corsano_ros_wrapper's convention)."""
    parser = argparse.ArgumentParser(description="Polar H10 ROS2 connector")
    parser.add_argument(
        "--mac-address-or-name",
        type=str,
        default=None,
        help="Device MAC address, or a name substring to match. Defaults to "
        "matching any device advertising as 'Polar H10'.",
    )
    parser.add_argument(
        "--bluetooth-adapter",
        type=str,
        default=None,
        help="Bluetooth adapter name, e.g. hci0",
    )
    for stream, default in (
        ("hr", True),
        ("acceleration", True),
        ("ecg", True),
        ("battery", True),
    ):
        parser.add_argument(
            f"--publish-{stream}",
            dest=f"publish_{stream}",
            action=argparse.BooleanOptionalAction,
            default=None,
            help=f"Publish {stream} data (default: {default}).",
        )
    clean_args = remove_ros_args(sys.argv)
    return parser.parse_args(clean_args[1:])


class PolarConnector(Node):

    def __init__(self):
        super().__init__("polar_connector")

        self.declare_parameter("publish_hr", True)
        self.declare_parameter("publish_acceleration", True)
        self.declare_parameter("publish_ecg", True)
        self.declare_parameter("publish_battery", True)
        self.declare_parameter("bluetooth_adapter", "hci0")
        self.declare_parameter("mac_address_or_name", "")
        self.pub_rr = self.create_publisher(Vector3Stamped, "polar_rr", 10)

    def start(self):
        self.publish_acceleration = (
            self.get_parameter("publish_acceleration").get_parameter_value().bool_value
        )
        self.publish_hr = (
            self.get_parameter("publish_hr").get_parameter_value().bool_value
        )
        self.publish_ecg = (
            self.get_parameter("publish_ecg").get_parameter_value().bool_value
        )
        self.publish_battery = (
            self.get_parameter("publish_battery").get_parameter_value().bool_value
        )
        self.bluetooth_adapter = (
            self.get_parameter("bluetooth_adapter").get_parameter_value().string_value
        )
        self.mac_address_or_name = (
            self.get_parameter("mac_address_or_name").get_parameter_value().string_value
        )

        loop_0 = asyncio.get_event_loop()
        while rclpy.ok():
            self.pd = None
            run_task = None
            try:
                self.pd = loop_0.run_until_complete(self.get_device())
                if self.pd is not None:
                    run_task = loop_0.create_task(self.run(self.pd))
                    loop_0.run_until_complete(run_task)
            except KeyboardInterrupt:
                log.info("[PolarConnector] Polar recording stopped by user.")
                break
            except Exception as e:
                log.error(f"[PolarConnector] Uncaught exception: {e}. Retrying...")
            finally:
                # Ctrl-C interrupts run_until_complete but leaves run() pending;
                # without the cancel, disconnect() below resumes it and it
                # publishes on the already-shut-down context ("publisher's
                # context is invalid" / "Task exception was never retrieved").
                if run_task is not None and not run_task.done():
                    run_task.cancel()
                    loop_0.run_until_complete(
                        asyncio.gather(run_task, return_exceptions=True)
                    )
                loop_0.run_until_complete(self.disconnect(self.pd))

    async def get_device(self):
        target = self.mac_address_or_name.upper() if self.mac_address_or_name else None
        while True:
            devices = await BleakScanner.discover()
            print(f"Devices: {devices}")
            for device in devices:
                if device.name is None or "Polar H10" not in device.name:
                    continue
                if target and target not in (device.address or "").upper() and (
                    target not in device.name.upper()
                ):
                    continue
                return PolarH10(
                    self,
                    device,
                    self.bluetooth_adapter,
                    self.publish_acceleration,
                    self.publish_hr,
                    self.publish_ecg,
                    self.publish_battery,
                )

            log.error("[PolarConnector] No matching Polar device found. Retrying...")

    async def run(self, polar_device):
        await polar_device.connect()
        log.info("[PolarConnector] Polar device is connected.")
        # await polar_device.get_device_info()
        # await polar_device.print_device_info()
        if self.publish_acceleration or self.publish_ecg:
            await polar_device.start_combined_stream()
        if self.publish_hr:
            await polar_device.start_hr_stream()

        waiting_since = None
        while True:
            await asyncio.sleep(1)
            acc_data = polar_device.get_acc_data()
            ibi_data = polar_device.get_ibi_data()
            if acc_data is None or ibi_data is None:
                # Expected right after connecting -- and this strap can take
                # ~32s to start ECG -- so say it once, then at most every 30s,
                # and never interpolate the arrays themselves: doing that turned
                # 441 warnings into ~4000 log lines in one run and buried every
                # line that mattered (2026-09-28 reconnect regression capture).
                missing = "acc" if acc_data is None else "RR"
                if waiting_since is None:
                    waiting_since = time.monotonic()
                    log.info(f"[PolarConnector] Waiting for {missing} data...")
                elif time.monotonic() - waiting_since >= 30:
                    log.warning(
                        f"[PolarConnector] Still no {missing} data after "
                        f"{time.monotonic() - waiting_since:.0f}s."
                    )
                    waiting_since = time.monotonic()
                continue
            waiting_since = None
            breathing_analyser = BreathingAnalyser(acc_data, ibi_data)
            msg = Vector3Stamped()
            msg.vector.x = float(breathing_analyser.get_rr())
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_rr.publish(msg)

    async def disconnect(self, polar_device):
        if polar_device is not None:
            if self.publish_acceleration or self.publish_ecg:
                await polar_device.stop_combined_stream()
            if self.publish_hr:
                await polar_device.stop_hr_stream()
            await polar_device.disconnect()


def main(args=None):
    # Without this the root logger drops INFO and prints WARNING+ with no
    # timestamp, so the driver log can't be lined up against the HCI capture.
    log.basicConfig(
        level=log.INFO,
        format="%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    cli_args = parse_args()
    rclpy.init(args=args)

    polar_node = PolarConnector()

    overrides = []
    if cli_args.mac_address_or_name is not None:
        overrides.append(
            Parameter(
                "mac_address_or_name", Parameter.Type.STRING, cli_args.mac_address_or_name
            )
        )
    if cli_args.bluetooth_adapter is not None:
        overrides.append(
            Parameter("bluetooth_adapter", Parameter.Type.STRING, cli_args.bluetooth_adapter)
        )
    for stream in ("hr", "acceleration", "ecg", "battery"):
        value = getattr(cli_args, f"publish_{stream}")
        if value is not None:
            overrides.append(Parameter(f"publish_{stream}", Parameter.Type.BOOL, value))
    if overrides:
        polar_node.set_parameters(overrides)

    polar_node.start()
    polar_node.destroy_node()
    # rclpy's SIGINT handler has usually shut the context down already;
    # a plain shutdown() then raises RCLError and the node exits with 1.
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
