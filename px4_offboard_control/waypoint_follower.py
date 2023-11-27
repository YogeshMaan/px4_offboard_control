import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus, SensorCombined
import math


class WaypointFollower(Node):
    "Node for controlling a vehicle in offboard mode using waypoints"

    def __init__(self) -> None: 
        super().__init__('waypoint_follower')

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
        self.sensor_combined_subscriber = self.create_subscription(
            SensorCombined, '/fmu/sensor_combined/out', self.sensor_combined_callback, 10)
        
        #create a timer_callback to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        

        #Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.sensor_combined = SensorCombined()

        # additional variables
    
        self.waypoints = self.generateWaypoints()
        self.wp_num = 0
        self.thres_error = .15
        self.collision_count = 0
        

    def generateWaypoints(self):
        self.get_logger().info("----Generating Waypoints----")
        wp = []
        # Wp 1
        wp.append([-0.5 , 0.0,-.75, 0.0 , 0.0,float("nan"), 0.0])
        # Wp 2
        wp.append([-0.5, 0.0 ,-.75, 0.0 , 0.0,float("nan"), 0.0])
        # Wp 3
        wp.append([float("nan"), float("nan"), -.75, 1.0, 0.0 ,float("nan"), 0.0])   # Velocities depend on angle of collision and ADD Yaw
        # Wp 4 (modified after collision)
        wp.append([.911, 0.0, -.75, float("nan"),float("nan"),float("nan"), 0.0])
        # Wp 5 (after mission over)
        wp.append([0.0, 0.0, -.75, float("nan"),float("nan"),float("nan"), 0.0])
        # Wp 6 (landing wp)
        wp.append([0.0, 0.0, 0.0, float("nan"),float("nan"),float("nan"), 0.0])

        print(wp)

        self.get_logger().info("----Waypoint generation completed!----")
        return wp
        

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry = vehicle_odometry

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def sensor_combined_callback(self, sensor_combined):
        self.sensor_combined = sensor_combined 
        
            
        
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

    def wp_publisher(self, wp_num):
        # add different cases to publish waypoints
        msg = TrajectorySetpoint()
        msg.x = self.waypoints[self.wp_num][0]
        msg.y = self.waypoints[self.wp_num][1]
        msg.z = self.waypoints[self.wp_num][2]
        msg.vx = self.waypoints[self.wp_num][3]
        msg.vy =  self.waypoints[self.wp_num][4]
        msg.vz =  self.waypoints[self.wp_num][5]
        msg.yaw = 0.0 # 0 degree
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[msg.x, msg.y, msg.z]}")

    
    def check(self):        

        if self.wp_num == 0:
            if self.euclideanError(self.wp_num) >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:        
                self.wp_num += 1
                self.wp_publisher(self.wp_num) 
        
        if self.wp_num == 1:
            if self.euclideanError(self.wp_num) >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:     
                self.wp_num += 1
                self.wp_publisher(self.wp_num) 

        if self.wp_num == 2:
            err2 = math.sqrt((self.vehicle_odometry.x - 1.25)**2 + 
                             (0)**2 + # made it zero
                             (0)**2)
            if err2 >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:     
                self.wp_num += 1
                self.wp_publisher(self.wp_num) 

        if self.wp_num == 3:
            if self.euclideanError(self.wp_num) >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:        
                self.wp_num += 1
                self.wp_publisher(self.wp_num)
        
        if self.wp_num == 4: #hover over landing wp
            if self.euclideanError(self.wp_num) >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:        
                self.wp_num += 1
                self.wp_publisher(self.wp_num)

        if self.wp_num == 5: # land
            if self.euclideanError(self.wp_num) >= self.thres_error:
                self.wp_publisher(self.wp_num)
            else:        
                self.land()
                
        return None


    def euclideanError(self, wp_i):
        err = math.sqrt((self.vehicle_odometry.x - self.waypoints[wp_i][0])**2 + 
                             (self.vehicle_odometry.y - self.waypoints[wp_i][1])**2 +
                             (self.vehicle_odometry.z - self.waypoints[wp_i][2])**2)
        return err
    
    
    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter +=1

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.check()



def main(args = None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)





