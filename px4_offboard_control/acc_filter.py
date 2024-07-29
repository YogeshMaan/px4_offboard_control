import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from MLPF import Median_LPF
from MV_Filter import Moving_Avg_Filter
import numpy as np

class Acc_Filter(Node):

    def __init__(self) -> None:
        super().__init__('acc_filter')

        #Subscriber 
        self.sensor_combined_subscriber = self.create_subscription(
            SensorCombined, '/fmu/sensor_combined/out', self.sensor_combined_callback, 10)
        
        #Publisher 
        self.filtered_sensor_combined_publisher = self.create_publisher(
            SensorCombined, 'filter_sensor_combined', 10 )

        self.sensor_combined = SensorCombined()
        self.ax_filter = Median_LPF()
        self.ay_filter = Median_LPF()
        self.alpha_ax = 0.4
        self.alpha_ay = 0.4
        self.window_size_idx = 9 # this parameter is len(window)-1
        self.a_filter_buffer = np.zeros((2,self.window_size_idx+1))

    def sensor_combined_callback(self, msg):
        self.sensor_combined = msg
        self.filter_data()


    def filter_data(self):
        #process and publish data 
        ax = self.sensor_combined.accelerometer_m_s2[0]
        ay = self.sensor_combined.accelerometer_m_s2[1]

        self.a_filter_buffer[0][:self.window_size_idx] = self.a_filter_buffer[0][1:]
        self.a_filter_buffer[0][self.window_size_idx]  = ax

        self.a_filter_buffer[1][:self.window_size_idx] = self.a_filter_buffer[1][1:]
        self.a_filter_buffer[1][self.window_size_idx]  = ay

        filtered_ax = self.ax_filter.med_lp(self.a_filter_buffer[0], self.alpha_ax)
        filtered_ay = self.ax_filter.med_lp(self.a_filter_buffer[1], self.alpha_ay)

        data = SensorCombined()
        data.accelerometer_m_s2[0] =filtered_ax
        data.accelerometer_m_s2[1] = filtered_ay
        data.timestamp = self.sensor_combined.timestamp

        self.filtered_sensor_combined_publisher.publish(data)



def main(args = None) -> None:
    print('Starting acc_filter node...')
    rclpy.init(args = args)
    acc_filter = Acc_Filter()
    rclpy.spin(acc_filter)
    acc_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)