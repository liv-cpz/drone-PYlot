colcon build --packages-select aruco_det
source install/setup.bash
ros2 launch aruco_det aruco_det.launch.py
ros2 run usb_cam usb_cam_node_exe
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /aruco/pose