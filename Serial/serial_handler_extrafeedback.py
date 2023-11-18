import rclpy
from rclpy.node import Node
import serial
from custom_interfaces.msg import SerialRead
from custom_interfaces.srv import SerialWrite
from collections import namedtuple

class SerialHandler(Node):
    def __init__(self):
        # Initialize node and topics/services
        super().__init__('serial_handler')
        self.publisher_ = self.create_publisher(SerialRead,'serial_read',10)
        self.write_srv_ = self.create_service(SerialWrite,'serial_write',self.serial_write_callback,10)

        # Open serial ports
        # I need to make a more sophisticated method to make sure that the portnames are correct
        self.SerialPort = namedtuple('SerialPort',['portname','ser'])
        self.SerialPorts[
            # Stand-in serial port names
            self.SerialPort('Actuators',serial.Serial('/dev/ttyACM0')),
            self.SerialPort('Sensors',serial.Serial('/dev/ttyACM1')),
        ]

        self.SerialWriteFeedback = namedtuple('SerialPortFeedback',['identifier','data'])
        self.serial_write_buffer= [] # Create an empty list for containing serial_write feedback

        timer_period  = 1/10 #s
        self.timer = self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        for i in self.SerialPorts:
            serial_read = self.SerialPorts[i].ser.readline() # Read the serial port
            serial_read = serial_read[:-2] # Trim off last two bytes which contain /r/n

            identifier_ = b''.join(serial_read[1:5]).decode("utf-8") # The identifier tells us what the data means
            data_ = serial_read[5:]
            
            if serial_read[1] == b'w': # If the first byte is 'w', add the data to the serial_write buffer
                self.serial_write_buffer.append(self.SerialWriteFeedback[identifier_,data_])
            else: # If the first byte is 'r' or anything else, perform serial_read logic
                msg = SerialRead()
                msg.identifier = identifier_
                msg.data = data_
                self.publisher_.publish(msg)
                print(msg.data) 

    def serial_write_callback(self,request,response):
        for i in self.SerialPorts:
            if self.SerialPort[i].portname == request.portname:
                port_to_write = self.SerialPorts[i].ser

        port_to_write.write(request.data)

        
def main():
    rclpy.init()

    serial_handler = SerialHandler()

    rclpy.spin(serial_handler)

    serial_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
