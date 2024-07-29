import rclpy 
from rclpy.node import Node
import rclpy.subscription
from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import SensorCombined, VehicleOdometry
import numpy as np
from scipy.spatial.transform import Rotation

class LocalSensorCombined(Node):
    
    def __init__(self) -> None:
        super().__init__('local_sensor_combined')
        #subcribers
        self.sensor_combined_subscriber = self.create_subscription(SensorCombined, 
                                                                 '/fmu/sensor_combined/out',
                                                                 self.sensor_combined_callback,
                                                                   10)
        self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry,
                                                                  '/fmu/vehicle_odometry/out',
                                                                   self.vehicle_odometry_callback, 10)

        #publisher
        self.local_orientation_publisher = self.create_publisher(Float64MultiArray, '/local_orientation', 10)
        self.local_orientation_publisher2 = self.create_publisher(Float64MultiArray, '/local_orientation2', 10)

        #parameters
        self.sensor_combined_msg = SensorCombined()
        self.vehicle_odometry_msg = VehicleOdometry()

    def sensor_combined_callback(self, msg1):
        self.sensor_combined_msg = msg1
        data_arr = Float64MultiArray()

        #extract acceleration data
        body_acc = np.empty((3,1))
        body_acc[0] = msg1.accelerometer_m_s2[0]
        body_acc[1] = -msg1.accelerometer_m_s2[1]
        body_acc[2] = -msg1.accelerometer_m_s2[2]

        # get quaternion and convert to scipy notation of quaternion
        quat = np.array([self.vehicle_odometry_msg.q[3],
                         self.vehicle_odometry_msg.q[0],
                         self.vehicle_odometry_msg.q[1],
                         self.vehicle_odometry_msg.q[2]])
        
        rot_mat = Rotation.from_quat(quat).as_matrix()
        rot_mat2 = Rotation.from_quat(quat)
        rot_euler = rot_mat2.as_euler('xyz', degrees=True)
        #convert acceleration to local frame
        loc_acc = np.array((3,1))
        # loc_acc = quat*body_acc*conj_quat
        loc_acc = np.matmul(rot_mat, body_acc)
        #data_arr.data = [float(loc_acc[0]), float(loc_acc[1]), float(loc_acc[2])]
        data_arr.data = [float(rot_euler[0]), float(rot_euler[1]), float(rot_euler[2])]
        print("local orientation:", data_arr)
        #publish data
        self.local_orientation_publisher.publish(data_arr)

        ##----------old method - -------------------------
        # Extract the values from Q
        q0 = self.vehicle_odometry_msg.q[0]
        q1 = self.vehicle_odometry_msg.q[1]
        q2 = self.vehicle_odometry_msg.q[2]
        q3 = self.vehicle_odometry_msg.q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix from body frame to world frame
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
        
        loc_acc2 = np.matmul(rot_matrix, body_acc)

        data_arr2 = Float64MultiArray()

        data_arr2.data = [float(loc_acc2[0]), float(loc_acc2[1]), float(loc_acc2[2])]

        self.local_orientation_publisher2.publish(data_arr2)

    def vehicle_odometry_callback(self, msg):
        print('something')
        self.vehicle_odometry_msg = msg

def main(args = None) -> None:
    print("Running Local sensor combined node...")
    rclpy.init(args = args)
    local_sensor_combined = LocalSensorCombined()
    rclpy.spin(local_sensor_combined)
    local_sensor_combined.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)