import rclpy
from rclpy.node import Node
import serial
import custom_interfaces
from custom_interfaces.msg import SerialRead
from custom_interfaces.srv import SerialWrite
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
			self.SerialPorts.append(self.SerialPort('Test',serial.Serial('/dev/ttyACM0',1000000)))
			print('Serial 1 Opened')
		except:
			pass
		try:
			self.SerialPorts.append(self.SerialPort('Test2',serial.Serial('/dev/ttyACM1')))
		except:
			pass
		
		timer_period  = 1/10 #s
		self.timer = self.create_timer(timer_period,self.timer_callback)

	def timer_callback(self):
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
				print(msg.data)
				print('\n')
				
				#if serial_read[1] == b'w': # If the first byte is 'w', add the data to the serial_write buffer
					#self.serial_write_buffer.append(self.SerialWriteFeedback[identifier_,data_])
				#else: # If the first byte is 'r' or anything else, perform serial_read logic
				
			else:
				pass

	def serial_write_callback(self,request,response):
		for port in self.SerialPorts:
			if port.portname == request.portname:
				port_to_write = port.ser

		try:
			request.data.append(b'\n')
			port_to_write.write(b''.join(request.data))
			print('Succesful Write')
			print(b''.join(request.data))
			print('\n')
		except:
			print('Failed Write')
		
		return response
		
def main():
	rclpy.init()

	serial_handler = SerialHandler()

	rclpy.spin(serial_handler)

	serial_handler.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
