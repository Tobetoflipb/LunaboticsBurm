import rclpy	# ROS
from rclpy.node import Node	
import serial	# Package for managing serial ports
from custom_interfaces.msg import SerialRead	# Interface for serial_read messages	
from custom_interfaces.srv import SerialWrite	# Interface for serial_write requests/responses
from collections import namedtuple

class SerialHandler(Node):
	def __init__(self):
		# Initialize node and topics/services
		super().__init__('serial_handler')
		self.publisher_ = self.create_publisher(SerialRead,'serial_read',10)
		self.write_srv_ = self.create_service(SerialWrite,'serial_write',self.serial_write_callback)

		# Open serial ports
		# I need to make a more sophisticated method to make sure that the portnames are correct
		self.SerialPort = namedtuple('SerialPort',['portname','ser'])
		self.SerialPorts = []
		try:
			self.SerialPorts.append(self.SerialPort('Test',serial.Serial('/dev/ttyACM0',1000000)))	# Create serial port on ACM0
			print('Serial 1 Opened')
		except:
			pass
		try:
			self.SerialPorts.append(self.SerialPort('Test2',serial.Serial('/dev/ttyACM1')))	# Create serial port on ACM1
		except:
			pass
		
		timer_period  = 1/10 #s
		self.timer = self.create_timer(timer_period,self.timer_callback) # Create serial read timer

	def timer_callback(self):	# Executes whenever the timer increments
		for port in self.SerialPorts:
			if port.ser.in_waiting > 0:
				serial_read = port.ser.readline() # Read the serial port
				serial_read = serial_read[:-2] # Trim off last two bytes which contain /r/n


				identifier_ = serial_read[0:4].decode("utf-8") # The identifier tells us what the data means
				data_ = serial_read[4:]

				msg = SerialRead()
				msg.identifier = identifier_
				msg.data = data_
				self.publisher_.publish(msg)

				print('Succesful Read')
				print(msg.data)	# Print published data for debug purposes
				print('\n')

	def serial_write_callback(self,request,response):
		for port in self.SerialPorts:
			# Set the serial port based on requested portname
			if port.portname == request.portname:
				port_to_write = port.ser

		try:
			request.data.append(b'\n')
			port_to_write.write(b''.join(request.data)) # Write the data to the requested serial port
			print('Succesful Write')
			print(b''.join(request.data))
			print('\n')
		except:
			print('Failed Write')
		
		return response
		
def main():
	rclpy.init() # Initialize ROS 

	serial_handler = SerialHandler() # Create an instance of the SerialHandler node

	rclpy.spin(serial_handler) # Spin the node (execute code)

	rclpy.shutdown() # Destroy Node


if __name__ == '__main__':
	main()
