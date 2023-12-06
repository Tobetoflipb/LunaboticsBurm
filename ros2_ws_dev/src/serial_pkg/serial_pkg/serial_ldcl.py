import rclpy
from rclpy.node import Node
from custom_interfaces.msg import SerialRead

class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_ldcl') # Initialize node with the name serial_ldcl
        self.subscriber_ = self.create_subscription(SerialRead,'serial_read',self.listener_callback,10) # Subscribe to the serial_read topic
        
        print('Load Cell Data:')

    def listener_callback(self,msg):    # Callback for when a serial_read msg is recieved
        if msg.identifier == 'ldcl':
            print('\n')
            print(msg.identifier)
            print(b''.join(msg.data).decode("utf-8"))   # Print recieved data

def main():
    rclpy.init()    # Initialize ROS 

    serial_listener = SerialListener()  # Create an instance of the SerialListener node

    rclpy.spin(serial_listener) # Spin the node (execute code)

    serial_listener.destroy_node()  # Destroy Node
    rclpy.shutdown()    # Destroy Node

if __name__ == '__main__':
    main()