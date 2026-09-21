import rclpy
from rclpy.node import Node
import cv2

from pupil_labs_ros.pupil_labs_neon_driver import PupilLabsNeonDriver

# Custom Healthcare Messages
from ros2_hc_msgs.msg import PupilGaze, Blink, Fixation, Saccade

# Standard ROS 2 Messages & Services
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

from pupil_labs.realtime_api.streaming.eye_events import (
    BlinkEventData, FixationEventData, FixationOnsetEventData,
)
from pupil_labs.realtime_api import DeviceError

class PupilLabsNeonRos(Node):
    def __init__(self, driver: PupilLabsNeonDriver):
        super().__init__('pupil_labs_driver')
        self.bridge = CvBridge()
        self.driver = driver
        
        # Wire up driver callbacks to ROS publishing methods
        self.driver.on_gaze = self._publish_gaze
        self.driver.on_video = self._publish_video
        self.driver.on_event = self._publish_event
        self.driver.on_imu = self._publish_imu

        # ==========================================
        # 1. PUBLISHERS
        # ==========================================
        self.gaze_pub_ = self.create_publisher(PupilGaze, 'biosensing/raw_biosignals/oculometrics/pupil_gaze', 10)
        self.blink_pub_ = self.create_publisher(Blink, 'biosensing/derived_biosignals/oculometrics/blink', 10)
        self.fixation_pub_ = self.create_publisher(Fixation, 'biosensing/derived_biosignals/oculometrics/fixation', 10)
        self.saccade_pub_ = self.create_publisher(Saccade, 'biosensing/derived_biosignals/oculometrics/saccade', 10)
        self.imu_pub_   = self.create_publisher(Imu, 'biosensing/raw_biosignals/imu/head_movement', 10)
        self.video_pub_ = self.create_publisher(Image, 'biosensing/raw_biosignals/oculometrics/scene_camera', qos_profile_sensor_data)
        self.battery_pub_ = self.create_publisher(Float32, 'hardware/biosensors/device/battery_level', 1)

        # ==========================================
        # 2. SERVICES & SUBSCRIBERS
        # ==========================================
        self.srv_start_rec = self.create_service(Trigger, 'pupil_labs/recording_start', self.start_recording_cb)
        self.srv_stop_rec  = self.create_service(Trigger, 'pupil_labs/recording_stop', self.stop_recording_cb)
        self.event_sub = self.create_subscription(String, 'pupil_labs/send_event', self.send_event_cb, 10)

        # Status polling
        self.status_timer = self.create_timer(10.0, self.publish_status)

    # ---------------------------------------------------------
    # CALLBACKS FROM DRIVER -> ROS PUBLISHERS
    # ---------------------------------------------------------
    def _publish_gaze(self, gaze):
        msg = PupilGaze()
        sec = int(gaze.timestamp_unix_seconds)
        nanosec = int((gaze.timestamp_unix_seconds - sec) * 1e9)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = "pupil_glasses"
        
        msg.gaze_x = float(gaze.x)
        msg.gaze_y = float(gaze.y)
        msg.worn = bool(gaze.worn)
        msg.pupil_diameter_right = float(gaze.pupil_diameter_right)
        msg.pupil_diameter_left = float(gaze.pupil_diameter_left)

        msg.eyelid_angle_top_left = float(gaze.eyelid_angle_top_left)
        msg.eyelid_angle_bottom_left = float(gaze.eyelid_angle_bottom_left)
        msg.eyelid_aperture_left = float(gaze.eyelid_aperture_left)
        msg.eyelid_angle_top_right = float(gaze.eyelid_angle_top_right)
        msg.eyelid_angle_bottom_right = float(gaze.eyelid_angle_bottom_right)
        msg.eyelid_aperture_right = float(gaze.eyelid_aperture_right)
        
        self.gaze_pub_.publish(msg)

    def _publish_video(self, matched):
        sec = int(matched.scene.timestamp_unix_seconds)
        nanosec = int((matched.scene.timestamp_unix_seconds - sec) * 1e9)

        # Overlay Gaze Circle on Video (Kept in ROS node as it's presentation logic)
        scene_img = matched.scene.bgr_pixels.copy()
        cv2.circle(
            scene_img,
            (int(matched.gaze.x), int(matched.gaze.y)),
            radius=80,
            color=(0, 0, 255),
            thickness=15,
        )

        # Overlay Eyes Video (Picture-in-Picture)
        if matched.eyes is not None:
            height, width, _ = matched.eyes.bgr_pixels.shape
            scene_img[:height, :width, :] = matched.eyes.bgr_pixels

        image_msg = self.bridge.cv2_to_imgmsg(scene_img, encoding="bgr8")
        image_msg.header.stamp.sec = sec
        image_msg.header.stamp.nanosec = nanosec
        image_msg.header.frame_id = "pupil_scene_camera"
        
        self.video_pub_.publish(image_msg)

    def _publish_event(self, eye_event):
        match eye_event:
            case FixationOnsetEventData():
                pass  # Onset only, no duration yet

            case BlinkEventData():
                msg = Blink()
                self._fill_header_from_event(msg, eye_event)
                msg.duration = float((eye_event.end_time_ns - eye_event.start_time_ns) / 1e9)
                msg.start_time_ns = int(eye_event.start_time_ns)
                msg.end_time_ns = int(eye_event.end_time_ns)
                self.blink_pub_.publish(msg)
                self.get_logger().info(f"[BLINK] Duration: {msg.duration:.3f}s")

            case FixationEventData() if eye_event.event_type == 1:
                msg = Fixation()
                self._fill_header_from_event(msg, eye_event)
                self._fill_kinematics(msg, eye_event)
                self.fixation_pub_.publish(msg)

            case FixationEventData() if eye_event.event_type == 0:
                msg = Saccade()
                self._fill_header_from_event(msg, eye_event)
                self._fill_kinematics(msg, eye_event)
                self.saccade_pub_.publish(msg)

    def _publish_imu(self, imu_data):
        msg = Imu()
        sec = int(imu_data.timestamp_unix_seconds)
        nanosec = int((imu_data.timestamp_unix_seconds - sec) * 1e9)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = "pupil_glasses_imu"
        
        msg.linear_acceleration.x = float(imu_data.accel_data.x)
        msg.linear_acceleration.y = float(imu_data.accel_data.y)
        msg.linear_acceleration.z = float(imu_data.accel_data.z)
        
        msg.angular_velocity.x = float(imu_data.gyro_data.x)
        msg.angular_velocity.y = float(imu_data.gyro_data.y)
        msg.angular_velocity.z = float(imu_data.gyro_data.z)
        
        msg.orientation.x = float(imu_data.quaternion.x)
        msg.orientation.y = float(imu_data.quaternion.y)
        msg.orientation.z = float(imu_data.quaternion.z)
        msg.orientation.w = float(imu_data.quaternion.w)
        
        self.imu_pub_.publish(msg)

    # ---------------------------------------------------------
    # HELPER METHODS (DRY)
    # ---------------------------------------------------------
    def _fill_header_from_event(self, msg, eye_event):
        ts_sec = eye_event.rtp_ts_unix_seconds
        ros_sec = int(ts_sec)
        ros_nanosec = int((ts_sec - ros_sec) * 1e9)
        msg.header.stamp.sec = ros_sec
        msg.header.stamp.nanosec = ros_nanosec
        msg.header.frame_id = "pupil_glasses"

    def _fill_kinematics(self, msg, eye_event):
        msg.duration = float((eye_event.end_time_ns - eye_event.start_time_ns) / 1e9)
        msg.start_time_ns = int(eye_event.start_time_ns)
        msg.end_time_ns = int(eye_event.end_time_ns)
        msg.start_gaze_x = float(eye_event.start_gaze_x)
        msg.start_gaze_y = float(eye_event.start_gaze_y)
        msg.end_gaze_x = float(eye_event.end_gaze_x)
        msg.end_gaze_y = float(eye_event.end_gaze_y)
        msg.mean_gaze_x = float(eye_event.mean_gaze_x)
        msg.mean_gaze_y = float(eye_event.mean_gaze_y)
        msg.amplitude_pixels = float(eye_event.amplitude_pixels)
        msg.amplitude_angle_deg = float(eye_event.amplitude_angle_deg)
        msg.mean_velocity = float(eye_event.mean_velocity)
        msg.max_velocity = float(eye_event.max_velocity)

    # ---------------------------------------------------------
    # REMOTE CONTROL & STATUS
    # ---------------------------------------------------------
    def publish_status(self):
        try:
            battery = Float32()
            battery.data = float(self.driver.get_battery_level())
            self.battery_pub_.publish(battery)
        except DeviceError as e:
            self.get_logger().warning(f"Could not read battery: {e}")

    def start_recording_cb(self, request, response):
        try:
            rec_id = self.driver.start_recording()
            response.success = True
            response.message = f"Recording started on phone with ID: {rec_id}"
            self.get_logger().info(response.message)
        except DeviceError as e:
            response.success = False
            response.message = f"Failed to start recording: {str(e)}"
        return response

    def stop_recording_cb(self, request, response):
        try:
            self.driver.stop_recording()
            response.success = True
            response.message = "Recording stopped and saved successfully."
            self.get_logger().info(response.message)
        except DeviceError as e:
            response.success = False
            response.message = f"Failed to stop recording: {str(e)}"
        return response

    def send_event_cb(self, msg):
        try:
            self.driver.send_event(msg.data)
            self.get_logger().info(f"Injected Timeline Event: {msg.data}")
        except DeviceError as e:
            self.get_logger().error(f"Failed to send event: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 1. The hardware driver is the Context Manager
        with PupilLabsNeonDriver() as driver:
            
            # 2. Pass the connected driver into the ROS node
            node = PupilLabsNeonRos(driver)
            
            try:
                # 3. Spin ROS
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
        # 4. Standard ROS teardown
        if node:
            node.destroy_node()
            
    except ConnectionError as e:
        print(f"Startup failed: {e}")
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
