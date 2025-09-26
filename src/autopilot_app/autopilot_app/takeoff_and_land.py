import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class AutopilotApp(Node):
    def __init__(self):
        super().__init__('autopilot_app')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.timer = self.create_timer(0.1, self.run_mission)
        self.mission_state = "INIT"
        self.takeoff_altitude = -10.0  # Use negative for altitude in NED frame
        self.get_logger().info("App Initialized. Starting mission sequence.")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # Offboard mode
        self.arm()
        self.mission_state = "TAKEOFF"
        self.get_logger().info(f"Taking off to {-self.takeoff_altitude} meters")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")
        self.mission_state = "LANDED"

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def run_mission(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_publisher.publish(offboard_msg)

        if self.mission_state == "INIT":
            self.takeoff()
        elif self.mission_state == "TAKEOFF":
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [float('nan'), float('nan'), self.takeoff_altitude]
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
            if self.get_clock().now().seconds_nanoseconds()[0] > 20: 
                 self.land()
        elif self.mission_state == "LANDED":
            self.get_logger().info("Mission Complete. Shutting down.")
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    app_node = AutopilotApp()
    rclpy.spin(app_node)
    app_node.destroy_node()

if __name__ == '__main__':
    main()
