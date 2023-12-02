import rclpy
from rclpy.node import Node
import serial
import std_msgs.msg
from custom_interfaces.msg import SerialRead
from custom_interfaces.srv import SerialWrite

class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')
        self.subscriber_ = self.create_subscription(SerialRead,'serial_read',self.listener_callback,10)
        self.cli = self.create_client(SerialWrite, 'serial_write')
        self.req = SerialWrite.Request()

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info('Serial Listener Begin')
        
        #self.timer = self.create_timer(2,self.timer_callback)
        

    def listener_callback(self,msg):
        print(msg.identifier)
        print(b''.join(msg.data))

    def timer_callback(self):
        self.send_request('Test',b'Test Data Written and Recieved')

    def send_request(self,portname,data):
        self.req.portname = portname
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        return self.future.result()

def main():
    rclpy.init()

    serial_listener = SerialListener()

    rclpy.spin(serial_listener)

    serial_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()