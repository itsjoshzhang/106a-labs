ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30

ros2 launch planning lab7_bringup.launch.py

A: 0.02042213540145424, B: 0.9968963910417469, C: -0.07602974361120482, D: -0.05295334790345538

colcon build; source install/setup.bash