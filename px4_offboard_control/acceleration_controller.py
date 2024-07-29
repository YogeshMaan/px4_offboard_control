import rclpy
from rclpy.Node import Node
from px4_msgs import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class AccelerationController(Node):

    def __init__(self) -> None:
        super().__init__('acceleration_controller')

        #publishers
        self.offboard_control_mode_publisher = rclpy.create_publisher()

        #subscribers



        #parameters


    #subscribers callback
        

    # publishers
        

    #publishing frequency for node
        


def main():
    print('starting acceleration controller node...')
    rclpy.init(args=args)
    acceleration_controller = AccelerationController()
    rclpy.spin(acceleration_controller)
    acceleration_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)