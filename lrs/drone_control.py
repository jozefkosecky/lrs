
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import math
import time

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.target_positions = [
            Pose(position = Point(x=0.0,y=0.0,z=3.0,), orientation =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0,)),
            Pose(position = Point(x=-2.0,y=0.0,z=3.0,), orientation =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0,)),
            Pose(position = Point(x=-2.0,y=0.0,z=1.0,), orientation =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0,)),
            Pose(position = Point(x=0.0,y=0.0,z=1.0,), orientation =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0,)),
            Pose(position = Point(x=0.0,y=0.0,z=3.0,), orientation =  Quaternion(x=0.0,y=0.0,z=0.0,w=1.0,)),
        ]

        self.index_of_target_position = 0

        self.get_logger().info('Program started')

        self.state_sub = self.create_subscription(State, 'mavros/state', self._state_cb, 10)

        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, 'mavros/cmd/land')

        self.position_target_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_pos_cb, qos_profile=qos_profile)

        self._set_mode()
        self._arming_dron()
        self._takeoff_dron()


        
    def _state_cb(self, msg: State):
        self.current_state = msg
        # self.get_logger().info(f'Current State: {self.current_state.mode}')

    def _local_pos_cb(self, msg: PoseStamped):
        current_local_pos = msg
        x = current_local_pos.pose.position.x
        y = current_local_pos.pose.position.y
        z = current_local_pos.pose.position.z
        
        
        target_position = self.target_positions[self.index_of_target_position]

        is_drone_at_target_position = self._check_position(x, y, z, target_position)

        if is_drone_at_target_position:
            if self.index_of_target_position + 1 < len(self.target_positions):
                self.index_of_target_position += 1
                self._publish_position_target(self.target_positions[self.index_of_target_position])
                is_drone_at_target_position = False
            else:
                self._land_dron()
        

                

        # self.get_logger().info(f'Current Local Position: \n x: {x}, \n y:{y}, \n z:{z}')

    def _check_position(self, current_x, current_y, current_z, target_position):
        target_x, target_y, target_z = target_position.position.x, target_position.position.y, target_position.position.z 
        threshold = 0.2 

        if self._euclidean_distance(current_x, current_y, current_z, target_x, target_y, target_z) < threshold:
            print("We are close enough to the target!")
            return True
        else:
            print("Still on the way to the target.")
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
        takeoff_req.altitude = 3.0

        # Service call
        response = self.takeoff_client.call_async(takeoff_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().success:
            self.get_logger().info('Successfully initiated takeoff.')
        else:
            self.get_logger().error('Failed to initiate takeoff.')
    
    def _land_dron(self):
        self.get_logger().info('Land...')

        # Wait for the takeoff service to be available
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for land service...')

        land_req = CommandTOL.Request()
        land_req.min_pitch = 0.0
        land_req.yaw = 90.0
        land_req.altitude = 0.0

        # Service call
        response = self.land_client.call_async(land_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().success:
            self.get_logger().info('Successfully initiated land.')
        else:
            self.get_logger().error('Failed to initiate land.')

    def _publish_position_target(self, position):
        # self._set_mode()
   
        self.get_logger().info(f'Current Local Position: \n x: {position.position.x}, \n y: {position.position.y}, \n z: {position.position.z}')

        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base')
        move_to_pose.pose = position


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