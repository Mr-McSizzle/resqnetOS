# In swarm_app/swarm_app/formation_flight.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class SwarmApp(Node):
    def __init__(self):
        super().__init__('swarm_app')
        self.get_logger().info("Swarm App Initialized.")
        
        # --- Drone Setup ---
        self.num_drones = 3
        self.drone_publishers = []
        self.takeoff_altitude = -5.0
        
        # Formation offsets (X, Y, Z) from a central point
        self.formation_offsets = [
            [0.0, -2.0, self.takeoff_altitude],  # Drone 1
            [0.0, 0.0, self.takeoff_altitude],   # Drone 2 (leader)
            [0.0, 2.0, self.takeoff_altitude],   # Drone 3
        ]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers for each drone, namespaced
        for i in range(self.num_drones):
            pubs = {
                'offboard': self.create_publisher(OffboardControlMode, f'/px4_{i+1}/fmu/in/offboard_control_mode', qos_profile),
                'trajectory': self.create_publisher(TrajectorySetpoint, f'/px4_{i+1}/fmu/in/trajectory_setpoint', qos_profile),
                'command': self.create_publisher(VehicleCommand, f'/px4_{i+1}/fmu/in/vehicle_command', qos_profile)
            }
            self.drone_publishers.append(pubs)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0

    def publish_vehicle_command(self, drone_index, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = drone_index + 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.drone_publishers[drone_index]['command'].publish(msg)

    def arm(self, drone_index):
        self.publish_vehicle_command(drone_index, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'Arm command sent to drone {drone_index + 1}')

    def set_offboard_mode(self, drone_index):
         self.publish_vehicle_command(drone_index, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
         self.get_logger().info(f'Set offboard mode command sent to drone {drone_index + 1}')

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            for i in range(self.num_drones):
                self.set_offboard_mode(i)
                self.arm(i)
        
        for i in range(self.num_drones):
            # Publish offboard control mode
            offboard_msg = OffboardControlMode()
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.drone_publishers[i]['offboard'].publish(offboard_msg)

            # Publish trajectory setpoint
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = self.formation_offsets[i]
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.drone_publishers[i]['trajectory'].publish(trajectory_msg)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    swarm_node = SwarmApp()
    rclpy.spin(swarm_node)
    swarm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
