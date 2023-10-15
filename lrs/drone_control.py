
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import math
import time

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.get_logger().info('Program started')

        self.state_sub = self.create_subscription(State, 'mavros/state', self._state_cb, 10)

        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')

        self.position_target_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_pos_cb, qos_profile=qos_profile)

        # self.current_state = None
        # # Wait for MAVROS SITL connection
        # while self.current_state is None or not self.current_state.connected:
        #     self.get_logger().info('Waiting for MAVROS SITL connection...')
        #     time.sleep(0.1)
        #
        # if self.current_state.mode != "GUIDED":
        #     self._set_mode()
        
        # if not self.current_state.armed:
        #     self._arming_dron()
        #     self._takeoff_dron()
        
        self._set_mode()
        self._arming_dron()
        self._takeoff_dron()

        self._publish_position_target()


        
    def _state_cb(self, msg: State):
        self.current_state = msg
        # self.get_logger().info(f'Current State: {self.current_state.mode}')

    def _local_pos_cb(self, msg: PoseStamped):
        current_local_pos = msg
        x = current_local_pos.pose.position.x
        y = current_local_pos.pose.position.y
        z = current_local_pos.pose.position.z
        # self.get_logger().info(f'Current Local Position: \n x: {x}, \n y:{y}, \n z:{z}')

    def _check_position(self, current_x, current_y, current_z):
        destination_x, destination_y, destination_z = ...  # your destination coordinates
        threshold = 0.2  # for example, adjust as needed

        if self._euclidean_distance(current_x, current_y, current_z, destination_x, destination_y, destination_z) < threshold:
            print("We are close enough to the destination!")
            return True
        else:
            print("Still on the way to the destination.")
            return False

    def _euclidean_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    def _set_mode(self):
        self.get_logger().info('Change mode to GUIDED.')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        guided_set_mode_req = SetMode.Request()
        guided_set_mode_req.base_mode = 0
        guided_set_mode_req.custom_mode = "GUIDED"

        # Service call
        response = self.set_mode_client.call_async(guided_set_mode_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().mode_sent:
            self.get_logger().info('Successfully changed mode to GUIDED.')
        else:
            self.get_logger().error('Failed to change mode to GUIDED.')

    def _arming_dron(self):
            self.get_logger().info('Arming drone...')

            # Wait for the arming service to be available
            while not self.arming_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for arming service...')

            arming_req = CommandBool.Request()
            arming_req.value = True

            # Service call
            response = self.arming_client.call_async(arming_req)

            rclpy.spin_until_future_complete(self, response)
            if response.result().success:
                self.get_logger().info('Successfully armed the drone.')
            else:
                self.get_logger().error('Failed to arm the drone.')

    def _takeoff_dron(self):
        self.get_logger().info('Taking off...')

        # Wait for the takeoff service to be available
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')

        takeoff_req = CommandTOL.Request()
        takeoff_req.min_pitch = 0.0
        takeoff_req.yaw = 90.0
        takeoff_req.altitude = 2.0

        # Service call
        response = self.takeoff_client.call_async(takeoff_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().success:
            self.get_logger().info('Successfully initiated takeoff.')
        else:
            self.get_logger().error('Failed to initiate takeoff.')

    def _publish_position_target(self):
        # self._set_mode()
        self.get_logger().info('Sending dron to pos...')
        # pose_msg = PoseStamped()
        # # Fill out the message fields based on a default pose
        # pose_msg.header.stamp = self.get_clock().now().to_msg()
        # pose_msg.header.frame_id = ""  # You can specify a frame_id if necessary
        # pose_msg.pose.position.x = 0.0
        # pose_msg.pose.position.y = 0.0
        # pose_msg.pose.position.z = 3.0
        # pose_msg.pose.orientation.x = 0.0
        # pose_msg.pose.orientation.y = 0.0
        # pose_msg.pose.orientation.z = 0.0
        # pose_msg.pose.orientation.w = 1.0  # This represents no rotation

        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base')
        move_to_pose.pose.position=Point(
                        x=0.0,
                        y=0.0,
                        z=3.0,
                    )
        move_to_pose.pose.orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0,
                    )

        # Publish the PoseStamped message
        self.position_target_pub.publish(move_to_pose)
        self.get_logger().info('PoseStamped message published!')
        

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()