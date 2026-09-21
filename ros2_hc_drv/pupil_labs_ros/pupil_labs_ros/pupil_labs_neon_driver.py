import threading
import time
from pupil_labs.realtime_api.simple import discover_one_device
from pupil_labs.realtime_api import DeviceError

class PupilLabsNeonDriver:
    """
    Standalone driver for Pupil Labs Neon. 
    Acts as a Context Manager to safely handle thread and connection lifecycles.
    """
    def __init__(self, max_search_duration_seconds=10):
        self.max_search_duration = max_search_duration_seconds
        self.device = None
        self._is_running = False
        self._threads = []
        
        # Callbacks
        self.on_gaze = None
        self.on_video = None
        self.on_event = None
        self.on_imu = None

    def __enter__(self):
        """Starts connections and threads automatically when using 'with'."""
        print("Looking for Pupil Labs device on network...")
        self.device = discover_one_device(max_search_duration_seconds=self.max_search_duration)
        
        if self.device is None:
            raise ConnectionError("No Pupil Labs device found. Check Wi-Fi connection.")
            
        print(f"Connected to Glasses: {self.device.serial_number_glasses} | Phone: {self.device.phone_name}")

        # Start streaming threads safely
        self._is_running = True
        self._threads = [
            threading.Thread(target=self._stream_gaze, daemon=True),
            threading.Thread(target=self._stream_video, daemon=True),
            threading.Thread(target=self._stream_events, daemon=True),
            threading.Thread(target=self._stream_imu, daemon=True)
        ]
        
        for t in self._threads:
            t.start()
            
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Ensures threads and connections are cleanly closed on exit or exception."""
        print("Shutting down hardware driver and stopping threads...")
        self._is_running = False
        
        # 1. Wait for threads to finish
        for t in self._threads:
            t.join(timeout=2.0)
            
        # 2. Close API connection
        if self.device:
            self.device.close()
            
        return False # Do not swallow exceptions

    # ---------------------------------------------------------
    # API Wrappers (Recording, Events, Battery)
    # ---------------------------------------------------------
    def get_battery_level(self):
        return self.device.battery_level_percent

    def start_recording(self):
        return self.device.recording_start()

    def stop_recording(self):
        self.device.recording_stop_and_save()

    def send_event(self, event_name):
        self.device.send_event(event_name, event_timestamp_unix_ns=int(time.time() * 1e9))

    # ---------------------------------------------------------
    # STREAMING THREADS
    # ---------------------------------------------------------
    def _stream_gaze(self):
        try:
            while self._is_running:
                gaze = self.device.receive_gaze_datum()
                if gaze and self.on_gaze:
                    self.on_gaze(gaze)
        except DeviceError as e:
            if self._is_running: print(f"Gaze stream error: {e}")

    def _stream_video(self):
        try:
            while self._is_running:
                matched = self.device.receive_matched_scene_and_eyes_video_frames_and_gaze()
                if matched and self.on_video:
                    self.on_video(matched)
        except DeviceError as e:
            if self._is_running: print(f"Matched video stream error: {e}")

    def _stream_events(self):
        try:
            while self._is_running:
                eye_event = self.device.receive_eye_events()
                if eye_event and self.on_event:
                    self.on_event(eye_event)
        except DeviceError as e:
            if self._is_running: print(f"Event thread error: {e}")

    def _stream_imu(self):
        try:
            while self._is_running:
                imu_data = self.device.receive_imu_datum()
                if imu_data and self.on_imu:
                    self.on_imu(imu_data)
        except DeviceError as e:
            if self._is_running: print(f"IMU thread error: {e}")
