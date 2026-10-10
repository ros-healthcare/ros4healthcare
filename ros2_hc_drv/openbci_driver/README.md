# openbci_driver

ROS 2 driver for OpenBCI Cyton and Ganglion boards using `healthcare_msgs/EEG` messages.

This package streams EEG data from OpenBCI devices into ROS 2, publishing raw EEG signals and device metadata in a format compatible with `healthcare_msgs`.

---

## Features

- Supports **OpenBCI Cyton** (8 or 16 channels with daisy) and Ganglion boards.
- Publishes EEG data using `healthcare_msgs/EEG`.
- Publishes EEG metadata using `healthcare_msgs/EEGInfo`.
- Supports **batched sample publishing** for efficient ROS 2 communication.
- Configurable via ROS 2 parameters: `port`, `channel_count`, `batch_size`.

---

## Installation

1. Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> openbci_driver
```
2. Install dependencies:
```
pip install pyserial numpy
```

3. Build the package:
```
cd ~/ros2_ws
colcon build --packages-select openbci_driver
source install/setup.bash
```

## Usage
Start the node:
```
ros2 run openbci_driver openbci_driver
```

## ROS 2 Parameters
| Parameter       | Type   | Default       | Description                         |
|-----------------|--------|---------------|-------------------------------------|
| port            | string | /dev/ttyUSB0  | Serial port of the OpenBCI device   |
| channel_count   | int    | 8             | Number of EEG channels              |
| batch_size      | int    | 10            | Number of samples per published message |


### Example:
```
ros2 run openbci_driver openbci_driver --ros-args -p port:=/dev/ttyUSB1 -p channel_count:=16 -p batch_size:=20
```

## Messages

- `EEG.msg`: Flattened EEG values and quality.

- `EEGInfo.msg`: Metadata including device info, montage, electrode types, preprocessing, and units.

All messages are defined in healthcare_msgs.

## Supported Boards

- Cyton: 8 channels (16 with daisy)

- Ganglion: 4 channels

*Ensure you set the correct channel_count and port for your device.*

## Contributing

Feel free to submit pull requests or open issues to support other devices/firmwares or fix bugs.

## License
MIT