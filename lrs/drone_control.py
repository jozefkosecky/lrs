
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
from mission import Mission, Task
import bisect
import numpy as np
import threading
from rclpy.executors import SingleThreadedExecutor
from mavros_msgs.msg import PositionTarget


class DroneControl(Node):
    def __init__(self, maps, bfs_algo):
        super().__init__('drone_control_node')
        self.maps = maps
        self.bfs_algo = bfs_algo
        self.pixels_distance = 0.05
        self.precision_hard = 0.05;
        self.precision_soft = 0.10;
        self.precision = self.precision_hard
        self.should_land = False
        self.running = True
        self.end_program = False

        self.is_perform_land_or_takeoff = True

        self.is_performing_task = False


        self.current_local_pos = Pose(position = Point(x=0.0,y=0.0,z=0.0,), orientation =  self._quaternion_from_angle_degrees(0))

        self.global_start_pos_of_dron = (220*self.pixels_distance, 285*self.pixels_distance)

        # Starting position. TakeOff.
        self.target_position = Pose(position = Point(x=0.0,y=0.0,z=2.0,), orientation =  self._quaternion_from_angle_degrees(0))
        self.start_pos_was_reached = False

        self.maps_altitude = np.array([25, 75, 80, 100, 125, 150, 175, 180, 200, 225])

        # self.target_positions = [
        #     Mission(x = 180*self.pixels_distance, y = 55*self.pixels_distance, z = 2.00, is_precision_hard = False, tasks = []),
        #     Mission(x = 90*self.pixels_distance, y = 55*self.pixels_distance, z = 2.25, is_precision_hard = True, tasks = [Task.LAND, Task.TAKEOFF]),
        #     Mission(x = 90*self.pixels_distance, y = 195*self.pixels_distance, z = 2.25, is_precision_hard = False, tasks = []),
        #     Mission(x = 230*self.pixels_distance, y = 200*self.pixels_distance, z = 2.00, is_precision_hard = False, tasks = []),
        #     Mission(x = self.global_start_pos_of_dron[0], y = self.global_start_pos_of_dron[1], z = 2.00, is_precision_hard = False, tasks = []),
        # ]

        self.target_positions = [
            Mission(x = 220*self.pixels_distance, y = 260*self.pixels_distance, z = 2.00, is_precision_hard = False, tasks = [Task.YAW90, Task.YAW180, Task.LAND, Task.TAKEOFF, Task.YAW270, Task.YAW0]),
            Mission(x = self.global_start_pos_of_dron[0], y = self.global_start_pos_of_dron[1], z = 2.00, is_precision_hard = False, tasks = []),
        ]

        self.trajectory = self._get_trajectory((0.0,0.0), self.target_positions[0])


        self.index_of_target_position = 0
        self.index_of_trajectory_target_position = 0

        self.print_msg_to_console('Program started')

        self.state_sub = self.create_subscription(State, 'mavros/state', self._state_cb, 10)

        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, 'mavros/cmd/land')

        self.position_target_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.land_target_pub = self.create_publisher(PoseStamped, 'mavros/landing_target/pose', 10)
        self.position_raw_target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_pos_cb, qos_profile=qos_profile)

        
        self._takeoff_dron()


    def main_loop(self):
        x = self.current_local_pos.position.x
        y = self.current_local_pos.position.y
        z = self.current_local_pos.position.z
        orientation = self.current_local_pos.orientation

        is_drone_at_target_position, distance = self._check_position(self.current_local_pos, self.target_position, self.precision)


        if not is_drone_at_target_position:
            if distance < 0.5 and not self.is_performing_task:
            # if distance < 0.5:
                self._publish_position_target(self.trajectory[self.index_of_trajectory_target_position])
            return
        
        self.is_performing_task = False
        if self.index_of_trajectory_target_position  == len(self.trajectory) - 1 and self.target_positions[self.index_of_target_position].tasks:
            self.precision = self.precision_soft
            self.is_performing_task = True
            task = self.target_positions[self.index_of_target_position].tasks[0]
            self._perform_task(task, x, y, z, orientation)
            return

        if self.index_of_trajectory_target_position + 1 < len(self.trajectory):
            # input("Pre pokracovanie na dalsiu poziciu stlac ENTER!")
            self.index_of_trajectory_target_position += 1
            self._publish_position_target(self.trajectory[self.index_of_trajectory_target_position])

            if self.index_of_trajectory_target_position == len(self.trajectory) - 1:
                self.precision = self.precision_hard if self.target_positions[self.index_of_target_position].is_precision_hard else self.precision_soft
                

        elif self.index_of_target_position + 1 < len(self.target_positions):
            # input("Pre pokracovanie na dalsiu poziciu stlac ENTER!")
        
            self.index_of_target_position += 1
            self.trajectory = self._get_trajectory((x,y), self.target_positions[self.index_of_target_position])

            self.index_of_trajectory_target_position = 0
            self._publish_position_target(self.trajectory[self.index_of_trajectory_target_position])

            self.precision = self.precision_soft

        else:
            # self._perform_task()
            self._land_dron()
            self.end_program = True

        self.target_position = self.trajectory[self.index_of_trajectory_target_position]
    
    def print_msg_to_console(self, msg):
        print(msg)
        # self.get_logger().info(msg)
        pass

    def _state_cb(self, msg: State):
        self.current_state = msg
        # self.get_logger().info(f'Current State: {self.current_state.mode}')

    def _local_pos_cb(self, msg: PoseStamped):
        current_local_pos = msg
        self.current_local_pos = current_local_pos.pose
       
        # self.print_msg_to_console(f'Current Local Position: \n x: {x}, \n y:{y}, \n z:{z}')
        # target_x, target_y, target_z = self.target_position.position.x, self.target_position.position.y, self.target_position.position.z 
        # self.print_msg_to_console(f'Destination \n x: {target_x}, \n y:{target_y}, \n z:{target_z}')

    def _perform_task(self, task, x_pos, y_pos, z_pos, orientation):
        
        if task is None:
            return
        
        if task == Task.LAND:
            if not self.current_state.armed:
                self.target_positions[self.index_of_target_position].tasks.pop(0)
            elif z_pos >= 0.3:
                self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=0.0,), orientation =  orientation)
                # self._publish_position_target(self.target_position)
                self._land_dron()
                return
            
        if task == Task.TAKEOFF:
            # time.sleep(5)
            self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=2.0,), orientation =  orientation)
            # self._publish_position_target(self.target_position)  
            self._takeoff_dron()
            self.target_positions[self.index_of_target_position].tasks.pop(0)
            return
        
        if task == Task.YAW0:
            self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=z_pos,), orientation =  self._quaternion_from_angle_degrees(0))
            self._publish_position_target(self.target_position) 
            self.target_positions[self.index_of_target_position].tasks.pop(0)
            return
        
        if task == Task.YAW90:
            self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=z_pos,), orientation =  self._quaternion_from_angle_degrees(-90))
            self._publish_position_target(self.target_position) 
            self.target_positions[self.index_of_target_position].tasks.pop(0)
            return

        if task == Task.YAW180:
            self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=z_pos,), orientation =  self._quaternion_from_angle_degrees(-180))
            self._publish_position_target(self.target_position) 
            self.target_positions[self.index_of_target_position].tasks.pop(0)
            return
        
        if task == Task.YAW270:
            self.target_position = Pose(position = Point(x=x_pos,y=y_pos,z=z_pos,), orientation =  self._quaternion_from_angle_degrees(-270))
            self._publish_position_target(self.target_position) 
            self.target_positions[self.index_of_target_position].tasks.pop(0)
            return
        

    def _get_trajectory(self, start_pos, target_pos):
        x_in_pixels = int(target_pos.x / self.pixels_distance)
        y_in_pixels = int(target_pos.y / self.pixels_distance)
        z_in_pixels = int(target_pos.z / self.pixels_distance)

        z_in_cm = target_pos.z * 100
        z = target_pos.z

        z_altitude = self.find_nearest_bigger(z_in_cm)

        start_pos_in_pixels = self._get_global_pos_in_pixels(start_pos)
        end_pos_in_pixels = (x_in_pixels, y_in_pixels)
        map = self.maps[str(z_altitude)]

        path = self.bfs_algo.get_trajectory(start_pos_in_pixels, end_pos_in_pixels, map)

        
        return [
            Pose(
                position=Point(x=local_pos[0], y=local_pos[1], z=z),
                orientation=self._quaternion_from_angle_degrees(0)
            )
            for pos in path
            for local_pos in [self._get_local_pos_from_pixels(pos)]
        ]
    
    def find_nearest_bigger(self, value):
        return self.maps_altitude[self.maps_altitude >= value].min() 

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


    def _check_position(self, current_local_pos, target_position, threshold = 0.2, orient_threshold=5):
        # if is_precision_hard:
        #     threshold = self.precision_hard
        # else:
        #     threshold = self.precision_soft

        current_x, current_y, current_z = current_local_pos.position.x, current_local_pos.position.y, current_local_pos.position.z
        target_x, target_y, target_z = target_position.position.x, target_position.position.y, target_position.position.z 
        
        distance = self._euclidean_distance(current_x, current_y, current_z, target_x, target_y, target_z)

        orien_current = self.quaternion_to_angle(current_local_pos.orientation)
        orien_target = self.quaternion_to_angle(target_position.orientation)

        angle_difference = self.angle_difference(orien_current, orien_target)

        if distance < threshold and angle_difference < orient_threshold:
            print("We are close enough to the target!")
            return True, distance
        else:
            print("Still on the way to the target.")
            return False, distance

    def _euclidean_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    
    def _quaternion_from_angle_degrees(self, angle_degrees):
        angle_radians = math.radians(angle_degrees) / 2.0
        w = math.cos(angle_radians)
        z = math.sin(angle_radians)

        return Quaternion(x=0.0,y=0.0,z=float(z), w=float(w),)
    
    def quaternion_to_angle(self, quat):
        # Calculate the angle in radians
        theta = 2 * math.acos(quat.w)
        # Convert to degrees
        angle_in_degrees = (theta * 180) / math.pi
        return angle_in_degrees
    
    def angle_difference(self, angle1, angle2):
        diff = abs(angle1 - angle2) % 360  # Use modulo to wrap around if over 360
        if diff > 180:
            diff = 360 - diff
        return diff

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
        self._set_mode()
        self._arming_dron()
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

    # def _publish_land_target(self, position):
    #     # self._set_mode()
    #     self.print_msg_to_console(f'Current Local Position: \n x: {position.position.x}, \n y: {position.position.y}, \n z: {position.position.z}')

    #     move_to_pose = PoseStamped()
    #     move_to_pose.header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base')
    #     move_to_pose.pose = position


    #     # Publish the PoseStamped message
    #     self.land_target_pub.publish(move_to_pose)
    #     self.print_msg_to_console('PoseStamped message published!')

    # def _publish_raw_position_target(self, position, vx=0.5, vy=0.5, vz=0.5):
    #     msg = PositionTarget()

    #     msg.header = Header(stamp=self.get_clock().now().to_msg())
    #     msg.coordinate_frame = 1
    #     msg.type_mask = (64 | 128 | 256 | 512 | 1024 | 2048)
        
    #     msg.position.x = position.position.x
    #     msg.position.y = position.position.y
    #     msg.position.z = position.position.z

    #     msg.velocity.x = vx
    #     msg.velocity.y = vy
    #     msg.velocity.z = vz

    #     msg.acceleration_or_force.x = 0.0
    #     msg.acceleration_or_force.y = 0.0
    #     msg.acceleration_or_force.z = 0.0

    #     msg.yaw = 0.0
    #     msg.yaw_rate = 0.0

    #     self.position_raw_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    maps = Maps()
    bfs_algo = BFS()
    drone_control = DroneControl(maps.maps, bfs_algo)
    # Start the periodic_check method in a separate thread
    # thread = threading.Thread(target=drone_control.periodic_check)
    # thread.start()

    while rclpy.ok() and not drone_control.end_program:
        drone_control.main_loop()
        rclpy.spin_once(drone_control)

    # Cleanup
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()