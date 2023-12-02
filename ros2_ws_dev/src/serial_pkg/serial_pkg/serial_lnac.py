import rclpy
from rclpy.node import Node
from custom_interfaces.msg import SerialRead


class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_lnac')
        self.subscriber_ = self.create_subscription(SerialRead,'serial_read',self.listener_callback,10)
        
        self.get_logger().info('Linear Actuator Data:')  

    def listener_callback(self,msg):
        if msg.identifier == 'lnac':
            print('\n')
            print(msg.identifier)
            print(b''.join(msg.data).decode("utf-8"))

def main():
    rclpy.init()

    serial_listener = SerialListener()

    rclpy.spin(serial_listener)

    serial_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()