import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import math

class PositionVelocityControl(Node):
    "Node for controlling a vehicle in offboard mode using position and velocity setpoints"

    def __init__(self) -> None:
        super().__init__('position_velocity_controller')

        #Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', 10)
        
        #Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, 'fmu/vehicle_odometry/out',self.vehicle_odometry_callback, 10)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, 10)

        #Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        #self.takeoff_height = -5.0
        self.waypoints = self.generate_waypoints()
        self.counter = 0
        self.err = 10 #initialise as high value
        self.thres_err = .15
        self.thres_delta_t = 30
        self.t_initial = self.get_clock().now().nanoseconds / 10**9
        #create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_waypoints(self):
        #generate waypoints for a rectangular trajectory
        self.get_logger().info("----Generating Waypoints----")
        wp = []
        wp.append([0.0,0.0,-.75, 0.0 , 0.0,float("nan")])

        '''
        # Cubic spline type waypoint generator
        for i in range(0, 600, 10):
            i = i/100
            x = .00055*i**3 + (.02)*(i**2)
            x_dot = (.00055*3)*i**2 + (.02*2)*i
            temp_arr = [x, 0.0, -.75, x_dot, 0.0, float('nan')]
            wp.append(temp_arr)
        '''
        wp.append([float("nan"), float("nan"), -.75, 0.3, 0.0 ,float("nan")])  
        wp.append([0.0, 0.0, -.75, float("nan"),float("nan"),float("nan")])
        self.get_logger().info("----Waypoint generation completed!----")
        return wp
        

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry = vehicle_odometry

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):            
        """send an arm command to the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z:float, vx:float, vy:float, vz:float):
        msg = TrajectorySetpoint()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.vx = vx
        msg.vy = vy
        msg.vz = vz
        msg.yaw = 0.0 # 0 degree
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter +=1

        delta_t = self.get_clock().now().nanoseconds/10**9 - self.t_initial

        # ---------Hover test--------
        # if self.vehicle_odometry.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # elif self.vehicle_odometry.z <= self.takeoff_height:
        #     self.land()
        #     exit(0)

        #------------------------------------
        #------Rectangular-------------------
        #------------------------------------    

        if self.err > self.thres_err and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.counter][0], self.waypoints[self.counter][1], self.waypoints[self.counter][2], self.waypoints[self.counter][3], self.waypoints[self.counter][4], self.waypoints[self.counter][5] )

        elif self.err <= self.thres_err and self.counter < len(self.waypoints) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.counter][0], self.waypoints[self.counter][1], self.waypoints[self.counter][2], self.waypoints[self.counter][3], self.waypoints[self.counter][4], self.waypoints[self.counter][5])
            self.counter +=1

        elif delta_t < self.thres_delta_t and self.counter == 1 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.counter][0], self.waypoints[self.counter][1], self.waypoints[self.counter][2], self.waypoints[self.counter][3], self.waypoints[self.counter][4], self.waypoints[self.counter][5])
            
        elif delta_t >= self.thres_delta_t and self.counter == 1 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.counter][0], self.waypoints[self.counter][1], self.waypoints[self.counter][2], self.waypoints[self.counter][3], self.waypoints[self.counter][4], self.waypoints[self.counter][5])            
            self.counter +=1

        elif self.err <= self.thres_err and self.counter >= len(self.waypoints):
            self.publish_position_setpoint(0.0, 0.0, -.75, float("nan"),float("nan"),float("nan"))  
                      
            exit(0)

        if self.counter < len(self.waypoints):
            self.err = math.sqrt((self.vehicle_odometry.x - self.waypoints[self.counter][0])**2 + (self.vehicle_odometry.y - self.waypoints[self.counter][1])**2 +(self.vehicle_odometry.z - self.waypoints[self.counter][2])**2)
    

        self.get_logger().info(f"Error: {self.err}")
        self.get_logger().info(f'Time(sec): {delta_t}')
        print("Testing 22")

def main(args = None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = PositionVelocityControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)





