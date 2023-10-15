import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class TemplateDroneControl(Node):

    def __init__(self):
        super().__init__('template_drone_control_node')
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSProfile.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos=qos_profile)
        
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')

        # Wait for MAVROS SITL connection
        while not hasattr(self, 'current_state') or not self.current_state.connected:
            time.sleep(0.1)

        guided_set_mode_req = SetMode.Request()
        guided_set_mode_req.custom_mode = "GUIDED"
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
            
        result_future = self.set_mode_client.call_async(guided_set_mode_req)

        # TODO: Test if drone state really changed to GUIDED
        # TODO: Arm and Take Off

        self.get_logger().info('Sending position command')
        # TODO: Implement position controller and mission commands here

    def local_pos_cb(self, msg: PoseStamped):
        current_local_pos = msg
        self.get_logger().info(f'Current Local Position: {current_local_pos.pose.position.x}, {current_local_pos.pose.position.y}, {current_local_pos.pose.position.z}')

    def state_cb(self, msg: State):
        self.current_state = msg
        self.get_logger().info(f'Current State: {self.current_state.mode}')

def main(args=None):
    rclpy.init(args=args)
    drone_control = TemplateDroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
