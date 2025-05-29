#!/usr/bin/env python3

""" Middleman between DJI Tello Python SDK and ROS2
    Modification of code from tentone: https://github.com/tentone/tello-ros2/tree/main
"""

import rclpy
import threading
import numpy
import time
import math
import av
import tf2_ros
import cv2
import yaml

from djitellopy import Tello

from rclpy.node import Node
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig, TelloMove, TelloRotate
from std_msgs.msg import Empty, UInt8, Bool, String, Int32
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python


# Convert a rotation from euler to quaternion.
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert rotation from quaternion to euler.
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]



class MiddleNode(Node):
    """Node used to translate DJI Tello information, status and commands to ROS2
    and back
    """
    def __init__(self):
        super().__init__("Middle Node")

        # ROS2 Params
        self.declare_parameter('connect_timeout', 10.0)
        self.declare_parameter('tello_ip', '192.168.10.1')
        self.declare_parameter('tf_base', 'map')
        self.declare_parameter('tf_drone', 'drone')
        self.declare_parameter('tf_pub', False)
        self.declare_parameter('camera_info_file', '')

        # Tello Connections
        self.connection_timeout = float(self.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.get_parameter('tello_ip').value)
        self.tf_base = str(self.get_parameter('tf_base').value)
        self.tf_drone = str(self.get_parameter('tf_drone').value)
        self.tf_pub = bool(self.get_parameter('tf_pub').value)
        self.camera_info_file = str(self.get_parameter('camera_info_file').value)

        self.camera_info = None

        # Check if camera info file was received as argument
        if len(self.camera_info_file) == 0:
            share_directory = ament_index_python.get_package_share_directory('drone-pylot-middleman')
            self.camera_info_file = share_directory + '/ost.yaml'


        with open(self.camera_info_file, 'r') as file:
            self.camera_info = yaml.load(file, Loader=yaml.FullLoader)

        
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connection_timeout)

        # Connect to drone
        self.get_logger().info('Tello: Connecting to drone')

        self.drone = Tello()
        self.drone.connect()

        # Connect to drone
        self.get_logger().info('Tello: Connection Success')


        # Init Publishers and Subscribers
        self.init_publishers()
        self.init_subscribers()

        # Processing threads
        self.start_video_capture()
        self.start_tello_status()
        self.start_tello_odom()


    def init_publishers(self):
        """Setup and initialise publishers
        Node publishes details from the Tello drone
        """
        self.pub_image_raw = self.create_publisher(Image, 'image_raw', 1)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.pub_status = self.create_publisher(TelloStatus, 'status', 1)
        self.pub_id = self.create_publisher(TelloID, 'id', 1)
        self.pub_imu = self.create_publisher(Imu, 'imu', 1)
        self.pub_battery = self.create_publisher(BatteryState, 'battery', 1)
        self.pub_temperature = self.create_publisher(Temperature, 'temperature', 1)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 1)

        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    
    def init_subscribers(self):
        """Setup and initialise subscribers
        Node subscribes to other ROS2 nodes and sends these details to 
        the Tello drone
        """
        self.sub_emergency = self.create_subscription(Empty, 'emergency', self.cb_emergency, 1)
        self.sub_takeoff = self.create_subscription(Empty, 'takeoff', self.cb_takeoff, 1)
        self.sub_land = self.create_subscription(Empty, 'land', self.cb_land, 1)
        self.sub_stop = self.create_subscription(Empty, 'stop', self.stop, 1)
        self.sub_reboot = self.create_subscription(Empty, 'reboot', self.reboot, 1)
        self.sub_control = self.create_subscription(Twist, 'control', self.cb_control, 1)
        self.sub_flip = self.create_subscription(String, 'flip', self.cb_flip, 1)
        self.sub_wifi_config = self.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1)
        self.sub_move_cmd = self.create_subscription(TelloMove, 'move', self.move_process, 1)
        self.sub_rotate_cmd = self.create_subscription(TelloRotate, 'rotate', self.rotate_process, 1)
        self.sub_speed = self.create_subscription(Int32, 'speed', self.set_speed, 1)


    def get_orientation_quaternion(self):
        """Get drone oridentation as a quaternion
        """
        deg_to_rad = math.pi / 180.0
        return euler_to_quaternion([
            self.drone.get_yaw() * deg_to_rad,
            self.drone.get_pitch() * deg_to_rad,
            self.drone.get_roll() * deg_to_rad
        ])


    def start_tello_odom(self, rate=1.0/10.0):
        """Start drone odometry thread
        Internal Method - Retrieves odom info from drone and publishes to ROS2
        Separate thread
        """
        def status_odom():
            """ Publish odom, TF and IMU info from tello
            """
            while True:
                # TF
                if self.tf_pub:
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.tf_base
                    t.child_frame_id = self.tf_drone
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = (self.drone.get_barometer()) / 100.0
                    self.tf_broadcaster.sendTransform(t)
                
                # IMU
                if self.pub_imu.get_subscription_count() > 0:
                    q = self.get_orientation_quaternion()

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    msg.linear_acceleration.x = self.drone.get_acceleration_x() / 100.0
                    msg.linear_acceleration.y = self.drone.get_acceleration_y() / 100.0
                    msg.linear_acceleration.z = self.drone.get_acceleration_z() / 100.0
                    msg.orientation.x = q[0]
                    msg.orientation.y = q[1]
                    msg.orientation.z = q[2]
                    msg.orientation.w = q[3]
                    self.pub_imu.publish(msg)

                # Odometry
                if self.pub_odom.get_subscription_count() > 0:
                    q = self.get_orientation_quaternion()

                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = self.tf_base
                    odom_msg.pose.pose.orientation.x = q[0]
                    odom_msg.pose.pose.orientation.y = q[1]
                    odom_msg.pose.pose.orientation.z = q[2]
                    odom_msg.pose.pose.orientation.w = q[3]
                    odom_msg.twist.twist.linear.x = float(self.drone.get_speed_x()) / 100.0
                    odom_msg.twist.twist.linear.y = float(self.drone.get_speed_y()) / 100.0
                    odom_msg.twist.twist.linear.z = float(self.drone.get_speed_z()) / 100.0
                    self.pub_odom.publish(odom_msg)
                
                time.sleep(rate)

        thread = threading.Thread(target=status_odom)
        thread.start()
        return thread


    def start_tello_status(self, rate=1.0/2.0):
        """Start drone Status thread
        Internal Method - Retrieves sensor info from drone and publishes to ROS2
        Separate thread
        """
        def status_loop():
            """ Publish Battery, Temp, Tello ID, Staus etc. info from tello
            """
            while True:
                # Battery
                if self.pub_battery.get_subscription_count() > 0:
                    msg = BatteryState()
                    msg.header.frame_id = self.tf_drone
                    msg.percentage = float(self.drone.get_battery())
                    msg.voltage = 3.8
                    msg.design_capacity = 1.1
                    msg.present = True
                    msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
                    msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
                    self.pub_battery.publish(msg)

                # Temperature
                if self.pub_temperature.get_subscription_count() > 0:
                    msg = Temperature()
                    msg.header.frame_id = self.tf_drone
                    msg.temperature = self.drone.get_temperature()
                    msg.variance = 0.0
                    self.pub_temperature.publish(msg)

                # Tello Status
                if self.pub_status.get_subscription_count() > 0:
                    msg = TelloStatus()
                    msg.acceleration.x = self.drone.get_acceleration_x()
                    msg.acceleration.y = self.drone.get_acceleration_y()
                    msg.acceleration.z = self.drone.get_acceleration_z()

                    msg.speed.x = float(self.drone.get_speed_x())
                    msg.speed.y = float(self.drone.get_speed_y())
                    msg.speed.z = float(self.drone.get_speed_z())

                    msg.pitch = self.drone.get_pitch()
                    msg.roll = self.drone.get_roll()
                    msg.yaw = self.drone.get_yaw()

                    msg.barometer = int(self.drone.get_barometer())
                    msg.distance_tof = self.drone.get_distance_tof()

                    msg.fligth_time = self.drone.get_flight_time()

                    msg.battery = self.drone.get_battery()

                    msg.highest_temperature = self.drone.get_highest_temperature()
                    msg.lowest_temperature = self.drone.get_lowest_temperature()
                    msg.temperature = self.drone.get_temperature()

                    msg.wifi_snr = self.drone.query_wifi_signal_noise_ratio()

                    self.pub_status.publish(msg)

                # Tello ID
                if self.pub_id.get_subscription_count() > 0:
                    msg = TelloID()
                    msg.sdk_version = self.drone.query_sdk_version()
                    msg.serial_number = self.drone.query_serial_number()
                    self.pub_id.publish(msg)

                # Camera info
                if self.pub_camera_info.get_subscription_count() > 0:
                    msg = CameraInfo()
                    msg.height = self.camera_info.image_height
                    msg.width = self.camera_info.image_width
                    msg.distortion_model = self.camera_info.distortion_model
                    msg.D = self.camera_info.distortion_coefficients
                    msg.K = self.camera_info.camera_matrix
                    msg.R = self.camera_info.rectification_matrix
                    msg.P = self.camera_info.projection_matrix
                    self.pub_camera_info.publish(msg)
                
                # Sleep
                time.sleep(rate)

        thread = threading.Thread(target=status_loop)
        thread.start()
        return thread


    # Start video capture thread.
    def start_video_capture(self, rate=1.0/30.0):
        """Start video capture thread
        """

        # Enable tello stream
        self.drone.streamon()

        # OpenCV bridge
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = self.drone.get_frame_read()

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'bgr8')
                msg.header.frame_id = self.tf_drone
                self.pub_image_raw.publish(msg)

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread

    # Terminate the code and shutdown node.
    def terminate(self, err):
        """terminate code and shutdown
        """
        self.get_logger().error(str(err))
        self.drone.end()
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        """Emergency stop drone
        """
        self.drone.emergency()

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        """Drone takeoff
        """
        self.drone.takeoff()

    # Land the drone message callback
    def cb_land(self, msg):
        """Land drone
        """
        self.drone.land()


    # More fine tune control --> expose more control
    def stop(self, msg):
        """Stop drone --> still in air
        """
        self.drone.stop()

    
    def move_process(self, msg):
        """Move drone - more control
        Based on message - distance to move and direction
        Message:
            direction: up, down, left, right, forward or back
            x: 20-500  
        """
        dist = msg.distance
        cmd = msg.movement_command

        if cmd == "up":
            self.move_up(dist)
        elif cmd == "down":
            self.move_down(dist)
        elif cmd == "left":
            self.move_left(dist)
        elif cmd == "right":
            self.move_right(dist)
        elif cmd == "forward":
            self.move_down(dist)
        elif cmd == "back":
            self.move_back(dist)
        else:
            self.get_logger().error("Invalid Command")


    def rotate_process(self, msg):
        """Rotate drone - more control
        Based on message - angle to move and direction
        Message:
            direction: clockwise, counter-clockwise
            x: 1-360  
        """
        angle = msg.angle
        cmd = msg.direction

        if cmd == "clockwise":
            self.rotate_clockwise(angle)
        elif cmd == "counter-clockwise":
            self.rotate_counter_clockwise(angle)
        else:
            self.get_logger().error("Invalid Command")


    def move_up(self, x):
        """Move drone up
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_up(x)

    def move_down(self, x):
        """Move drone down
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_down(x)

    def move_left(self, x):
        """Move drone left
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_left(x)

    def move_right(self, x):
        """Move drone right
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_right(x)
    
    def move_forward(self, x):
        """Move drone forward
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_forward(x)

    def move_back(self, x):
        """Move drone back
        Internal command
        Arguments:
            x: distance (20-500 cm)
        """
        self.drone.move_back(x)

    def rotate_clockwise(self, x):
        """Rotate clockwise
        Internal Command
        Arguments:
            x: 1-360 degrees
        """
        self.drone.rotate_clockwise(x)

    def rotate_counter_clockwise(self, x):
        """Rotate counter-clockwise
        Internal Command
        Arguments:
            x: 1-360 degrees
        """
        self.drone.rotate_counter_clockwise(x)


    def set_speed(self, x):
        """Set drone speed cm/s
        Arguments:
            x cm/s
        """
        self.drone.set_speed(x)

    
    def reboot(self, msg):
        """Reboot
        """
        self.drone.reboot()


    # @todo --> check if all queries used?


    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        """Control drone via Twist Message
        """
        self.drone.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))


    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    def cb_wifi_config(self, msg):
        """Wifi Credentials
        """
        self.drone.set_wifi_credentials(msg.ssid, msg.password)
    

    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        """Flip drone
        """
        self.drone.flip(msg.data)



def main(args=None):
    rclpy.init(args=args)

    drone = MiddleNode()

    rclpy.spin(drone)

    drone.cb_shutdown()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()