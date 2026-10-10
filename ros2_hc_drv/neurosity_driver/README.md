# Neurosity Crown EEG Driver for ROS 2

ROS 2 Python driver to stream EEG data from Neurosity devices and publish it using standardized `healthcare_msgs` messages.

## Features

- Wraps the official [Neurosity SDK](https://github.com/neurosity/neurosity-sdk-python)  
- Publishes raw or filtered EEG streams as `healthcare_msgs/EEG`  
- Publishes metadata as `healthcare_msgs/EEGInfo`  
- Supports dynamic detection of channel names and sample sizes  
- Compatible with any Neurosity device supporting the SDK  

---

## Prerequisites

- ROS 2 (tested with Humble / Iron)  
- Python 3.10+  
- Neurosity SDK: `pip install neurosity-sdk`  
- Python dotenv: `pip install python-dotenv`  

Your ROS workspace should also include the `healthcare_msgs` package with `EEG` and `EEGInfo` message definitions.

---

## Installation

1. Clone the repository into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/neurosity_eeg_driver.git

```

2. Install dependencies:
```
pip3 install neurosity-sdk python-dotenv
```

Build the workspace:
```
cd ~/ros2_ws
colcon build --packages-select healthcare_msgs neurosity_eeg_driver
source install/setup.bash
```

## Environment Variables

The driver requires three environment variables to connect to your Neurosity device. Create a .env file in your workspace or home directory:

# ~/.env or neurosity_driver/.env
```
NEUROSITY_DEVICE_ID=your_device_id
NEUROSITY_EMAIL=your_email@example.com
NEUROSITY_PASSWORD=your_password
```

Load the variables before running ROS:
```
export $(cat ~/.env | xargs)  # or use `source ~/.env`
```

Alternatively, if the .env file is placed in the driver package folder, it will be automatically loaded by python-dotenv.

## Running the Driver

Launch the ROS 2 node using the provided launch file:
```
ros2 run neurosity_driver neurosity_driver
```

- `neurosity/eeg` topic publishes `healthcare_msgs/EEG` messages

- `neurosity/eeg_info` topic publishes `healthcare_msgs/EEGInfo` messages

You can echo the data in another terminal:

``` 
ros2 topic echo /neurosity/eeg
ros2 topic echo /neurosity/eeg_info
```
## Example Usage
You can subscribe to the eeg messages and process them:
```
import rclpy
from rclpy.node import Node
from healthcare_msgs.msg import EEG

class EEGListener(Node):
    def __init__(self):
        super().__init__('eeg_listener')
        self.sub = self.create_subscription(EEG, '/neurosity/eeg', self.callback, 10)

    def callback(self, msg):
        print(f"Received EEG with {msg.sample_size} samples per channel")

rclpy.init()
node = EEGListener()
rclpy.spin(node)
rclpy.shutdown()
``` 

## Notes

Neurosity SDK returns brainwave (EEG) data, not ECG.

The driver publishes *flattened arrays* (channel_count * sample_count) in EEG.eeg.

EEG *quality* is currently a *placeholder* (1.0 for all channels) — the SDK doesn’t provide per-channel signal quality.

## Contributing

Contributions welcome! Please open issues or PRs to improve support or fix bugs.

## License

MIT