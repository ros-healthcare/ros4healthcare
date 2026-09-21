cmake_minimum_required(VERSION 3.8)
project(ros2_hc_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(msg_files

"msg/biosensing/derived_biosignals/rr/RRHeader.msg"
"msg/biosensing/derived_biosignals/rr/RR.msg"
"msg/biosensing/derived_biosignals/hrv/HRV.msg"
"msg/biosensing/derived_biosignals/hrv/HRVHeader.msg"
"msg/biosensing/derived_biosignals/co/COHeader.msg"
"msg/biosensing/derived_biosignals/co/CO.msg"
"msg/biosensing/derived_biosignals/hr/HRHeader.msg"
"msg/biosensing/derived_biosignals/hr/HR.msg"
"msg/biosensing/derived_biosignals/sv/SV.msg"
"msg/biosensing/derived_biosignals/sv/SVHeader.msg"
"msg/biosensing/raw_biosignals/bcg/BCG.msg"
"msg/biosensing/raw_biosignals/bcg/BCGHeader.msg"
"msg/biosensing/raw_biosignals/ecg/ECGLeads.msg"
"msg/biosensing/raw_biosignals/ecg/ECGLead.msg"
"msg/biosensing/raw_biosignals/ecg/ECGHeader.msg"
"msg/biosensing/raw_biosignals/eeg/EEGHeader.msg"
"msg/biosensing/raw_biosignals/eeg/EEGChannels.msg"
"msg/biosensing/raw_biosignals/eeg/EEGChannel.msg"
"msg/biosensing/raw_biosignals/eda/EDAHeader.msg"
"msg/biosensing/raw_biosignals/eda/EDA.msg"
"msg/biosensing/raw_biosignals/emg/EMG.msg"
"msg/biosensing/raw_biosignals/emg/EMGHeader.msg"
"msg/biosensing/raw_biosignals/ppg/PPG.msg"
"msg/biosensing/raw_biosignals/ppg/PPGHeader.msg"
"msg/biosensing/raw_biosignals/ppg/PPGs.msg"
"msg/biosensing/raw_biosignals/icg/ICGLead.msg"
"msg/biosensing/raw_biosignals/icg/ICGHeader.msg"
"msg/biosensing/raw_biosignals/icg/ICGLeads.msg"
"msg/biosensing/raw_biosignals/eog/EOGHeader.msg"
"msg/biosensing/raw_biosignals/eog/EOG.msg"
"msg/biosensing/raw_biosignals/oculometrics/PupilGaze.msg"
"msg/biosensing/derived_biosignals/oculometrics/Blink.msg"
"msg/biosensing/derived_biosignals/oculometrics/Fixation.msg"
"msg/biosensing/derived_biosignals/oculometrics/Saccade.msg"

"msg/physical_sensors/external_signals/force/Force.msg"
"msg/physical_sensors/external_signals/pressure/Pressure.msg"
"msg/physical_sensors/external_signals/pressure/PressureHeader.msg"
"msg/physical_sensors/derived_signals/joint_angular_velocity/JointAngularVelocity.msg"
"msg/physical_sensors/derived_signals/joint_moments/JointMoments.msg"
"msg/physical_sensors/derived_signals/joint_angles/JointAngles.msg"
"msg/physical_sensors/derived_signals/pose/Pose.msg"
"msg/physical_sensors/derived_signals/elevation_angle/ElevationAngle.msg"
"msg/physical_sensors/derived_signals/steps/Step.msg"


"msg/hardware/biosensors/device/Device.msg"
"msg/hardware/biosensors/filter/Filter.msg"


"msg/biometrics/physiological/posture/PostureHeader.msg"
"msg/biometrics/physiological/posture/Posture.msg"
"msg/biometrics/physiological/gait/Gait.msg"
"msg/biometrics/physiological/adl/ADLHeader.msg"
"msg/biometrics/physiological/adl/ADL.msg"
"msg/biometrics/behavioral/mood/Mood.msg"

)

# Generate ROS interfaces
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(LD_LIBRARY_PATH "${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")

# Install package metadata
ament_package()
