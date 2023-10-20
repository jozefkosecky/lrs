
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import math
import time
from bfs_algorithm.maps import Maps
from bfs_algorithm.bfs import BFS
from mission import Mission
import bisect
import numpy as np

class DroneControl(Node):
    def __init__(self, maps, bfs_algo):
        super().__init__('drone_control_node')
        self.maps = maps
        self.bfs_algo = bfs_algo
        self.pixels_distance = 0.05

        self.global_start_pos_of_dron = (225*self.pixels_distance, 285*self.pixels_distance)

        self.start_pos = Pose(position = Point(x=0.0,y=0.0,z=2.0,), orientation =  self._quaternion_from_angle_degrees(0))
        self.start_pos_was_reached = False

        self.maps_altitude = np.array([25, 75, 80, 100, 125, 150, 175, 180, 200, 225])

        self.target_positions = [
            Mission(x = 180*self.pixels_distance, y = 55*self.pixels_distance, z = 2.00, precision_hard = False, task = "land"),
            Mission(x = self.global_start_pos_of_dron[0], y = self.global_start_pos_of_dron[1], z = 2.00, precision_hard = False, task = "land"),
        ]

        self.trajectory = self._get_trajectory((0.0,0.0), self.target_positions[0])

        print("hej")

        # self.trajectory = [
        #     Pose(position = Point(x=pos1_x,y=pos1_y,z=3.0,), orientation =  self._quaternion_from_angle_degrees(0)),
        #     Pose(position = Point(x=pos2_x,y=pos2_y,z=3.0,), orientation =  self._quaternion_from_angle_degrees(180)),
        #     # Pose(position = Point(x=-2.0,y=0.0,z=1.0,), orientation =  self._quaternion_from_angle_degrees(270)),
        #     # Pose(position = Point(x=0.0,y=0.0,z=1.0,), orientation =  self._quaternion_from_angle_degrees(45)),
        #     # Pose(position = Point(x=0.0,y=0.0,z=3.0,), orientation =  self._quaternion_from_angle_degrees(0)),
        # ]


        self.index_of_target_position = 0
        self.index_of_trajectory_target_position = 0

        self.print_msg_to_console('Program started')

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


    
    def print_msg_to_console(self, msg):
        print(msg)
        # self.get_logger().info(msg)
        pass

    def _state_cb(self, msg: State):
        self.current_state = msg
        # self.get_logger().info(f'Current State: {self.current_state.mode}')

    def _local_pos_cb(self, msg: PoseStamped):
        current_local_pos = msg
        x = current_local_pos.pose.position.x
        y = current_local_pos.pose.position.y
        z = current_local_pos.pose.position.z
        
        if not self.start_pos_was_reached:
            target_position = self.start_pos
        else:
            target_position = self.trajectory[self.index_of_trajectory_target_position]

        is_drone_at_target_position = self._check_position(x, y, z, target_position, 0.3)

        if is_drone_at_target_position:
            self.start_pos_was_reached = True

            if self.index_of_trajectory_target_position + 1 < len(self.trajectory):
                self.index_of_trajectory_target_position += 1
                input("Pre pokracovanie na dalsiu poziciu stlac ENTER!")
                self._publish_position_target(self.trajectory[self.index_of_trajectory_target_position])
                is_drone_at_target_position = False
            elif self.index_of_target_position + 1 < len(self.target_positions):
                self.index_of_target_position += 1
                self.trajectory = self._get_trajectory((x,y), self.target_positions[self.index_of_target_position])
                self.index_of_trajectory_target_position = 0
            else:
                self._land_dron()
        
        # self.get_logger().info(f'Current Local Position: \n x: {x}, \n y:{y}, \n z:{z}')

    def _get_trajectory(self, start_pos, target_pos):
        x_in_pixels = int(target_pos.x / self.pixels_distance)
        y_in_pixels = int(target_pos.y / self.pixels_distance)
        z_in_pixels = int(target_pos.z / self.pixels_distance)

        z_altitude = self.find_nearest_bigger(z_in_pixels)

        start_pos_in_pixels = self._get_global_pos_in_pixels(start_pos)
        end_pos_in_pixels = (x_in_pixels, y_in_pixels)
        map = self.maps[str(z_altitude)]

        path = self.bfs_algo.get_trajectory(start_pos_in_pixels, end_pos_in_pixels, map)

        z = target_pos.z
        return [
            Pose(
                position=Point(x=local_pos[0], y=local_pos[1], z=z),
                orientation=self._quaternion_from_angle_degrees(0)
            )
            for pos in path
            for local_pos in [self._get_local_pos_from_pixels(pos)]
        ]
    
    def find_nearest_bigger(self, value):
        return self.maps_altitude[self.maps_altitude > value].min() 

    def find_nearest_lower(self, value):
        return self.maps_altitude[self.maps_altitude < value].max() 
    
    def _get_local_pos_from_pixels(self, global_pos):
        x = global_pos[0] * self.pixels_distance
        y = global_pos[1] * self.pixels_distance
        return x - self.global_start_pos_of_dron[0], y - self.global_start_pos_of_dron[1]
    
    def _get_global_pos_in_pixels(self, local_pos):
        x_global = local_pos[0] + self.global_start_pos_of_dron[0]
        y_global = local_pos[1] + self.global_start_pos_of_dron[1]

        x_pixels = x_global / self.pixels_distance
        y_pixels = y_global / self.pixels_distance

        return int(x_pixels), int(y_pixels)
    
    def _get_local_pos(self, global_pos):
        return global_pos[0] - self.global_start_pos_of_dron[0], global_pos[1] - self.global_start_pos_of_dron[1]
    
    def _get_global_pos(self, local_pos):
        return local_pos[0] + self.global_start_pos_of_dron[0], local_pos[1] + self.global_start_pos_of_dron[1]


    def _check_position(self, current_x, current_y, current_z, target_position, threshold = 0.2):
        target_x, target_y, target_z = target_position.position.x, target_position.position.y, target_position.position.z 
        

        if self._euclidean_distance(current_x, current_y, current_z, target_x, target_y, target_z) < threshold:
            print("We are close enough to the target!")
            return True
        else:
            print("Still on the way to the target.")
            return False

    def _euclidean_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    
    def _quaternion_from_angle_degrees(self, angle_degrees):
        angle_radians = math.radians(angle_degrees) / 2.0
        w = math.cos(angle_radians)
        z = math.sin(angle_radians)

        return Quaternion(x=0.0,y=0.0,z=float(z), w=float(w),)

    def _set_mode(self):
        self.print_msg_to_console('Change mode to GUIDED.')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.print_msg_to_console('Waiting for set_mode service...')

        guided_set_mode_req = SetMode.Request()
        guided_set_mode_req.base_mode = 0
        guided_set_mode_req.custom_mode = "GUIDED"

        # Service call
        response = self.set_mode_client.call_async(guided_set_mode_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().mode_sent:
            self.print_msg_to_console('Successfully changed mode to GUIDED.')
        else:
            self.print_msg_to_console('Failed to change mode to GUIDED.')

    def _arming_dron(self):
            self.print_msg_to_console('Arming drone...')

            # Wait for the arming service to be available
            while not self.arming_client.wait_for_service(timeout_sec=1.0):
                self.print_msg_to_console('Waiting for arming service...')

            arming_req = CommandBool.Request()
            arming_req.value = True

            # Service call
            response = self.arming_client.call_async(arming_req)

            rclpy.spin_until_future_complete(self, response)
            if response.result().success:
                self.print_msg_to_console('Successfully armed the drone.')
            else:
                self.print_msg_to_console('Failed to arm the drone.')

    def _takeoff_dron(self):
        self.print_msg_to_console('Taking off...')

        # Wait for the takeoff service to be available
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.print_msg_to_console('Waiting for takeoff service...')

        takeoff_req = CommandTOL.Request()
        takeoff_req.min_pitch = 0.0
        takeoff_req.yaw = 90.0
        takeoff_req.altitude = 2.0

        # Service call
        response = self.takeoff_client.call_async(takeoff_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().success:
            self.print_msg_to_console('Successfully initiated takeoff.')
        else:
            self.print_msg_to_console('Failed to initiate takeoff.')
    
    def _land_dron(self):
        self.print_msg_to_console('Land...')

        # Wait for the takeoff service to be available
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.print_msg_to_console('Waiting for land service...')

        land_req = CommandTOL.Request()
        land_req.min_pitch = 0.0
        land_req.yaw = 90.0
        land_req.altitude = 0.0

        # Service call
        response = self.land_client.call_async(land_req)

        rclpy.spin_until_future_complete(self, response)
        if response.result().success:
            self.print_msg_to_console('Successfully initiated land.')
        else:
            self.print_msg_to_console('Failed to initiate land.')

    def _publish_position_target(self, position):
        # self._set_mode()
        self.print_msg_to_console(f'Current Local Position: \n x: {position.position.x}, \n y: {position.position.y}, \n z: {position.position.z}')

        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base')
        move_to_pose.pose = position


        # Publish the PoseStamped message
        self.position_target_pub.publish(move_to_pose)
        self.print_msg_to_console('PoseStamped message published!')
        

def main(args=None):
    rclpy.init(args=args)
    maps = Maps()
    bfs_algo = BFS()
    drone_control = DroneControl(maps.maps, bfs_algo)
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()