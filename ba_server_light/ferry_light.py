import serial 
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class Subscriber(Node):
	# Defining the serial port connected to relay board 
    Serial_port = '/dev/ttyUSB0'  #Connected to first usb port on the pi

    # Command for the relay board in integer - (byte 0)
    cmd_initial = 1  # initializing the relay board 
    cmd_activate = 6 # Activiating the relay
    cmd_toggle = 8  # toggle the light 
    
   	# Address of the relay board - (byte 1)
   	address_relay = 1 # Only one relay board (integer)
    
   	# Defining the data to the relay board - (byte 2) 
    Dock_light = 0b10000000   # First relay output for docking light
    Nav_light = 0b0100000    # Second relay output for navigation light
   
    # Xor sum for activating single relay - (byte 3)
    Dock_single =(cmd_activate^address_relay^Dock_light) # Single activation of Docking lights
    Nav_single = (cmd_activate^address_relay^Nav_light)	 # Single activation of Navigation light
 
    # XOR sum for toggle - (byte 3) 
    Dock_XOR = (cmd_toggle^address_relay^Dock_light) # XOR for docking light output
    Nav_XOR =  (cmd_toggle^address_relay^Nav_light)  # XOR for navigation light output

    def __init__(self):

        # Node for the Ferry light
        super().__init__('light_subscriber')

        # Checking the serial port 
		if not os.path.exists(self.Serial_port):
			self.get_logger().error("Serial port does not exists :" + self.Serial_port)
			self.get_logger().info("Command not sent to the relay board")
			rclpy.shutdown()

        	# Start the serial communication
        self.ser = serial.Serial(
            port=self.Serial_port, 
            baudrate=19200, 
            parity= serial.PARITY_NONE, 
            stopbits = serial.STOPBITS_ONE, 
            bytesize= serial.EIGHTBITS,
            timeout=10)   # Serial data configuration based on Conrad manual 
        
        	# Checking the serial port is open 
        try: 
			self.ser.isOpen()==False
            self.ser.open()      # Opens the serial port

		except:
			print('Serial port is already open') 

        # Creating Boolean subscription 
        self.subscription = self.create_subscription(Bool,'/Bool_topic', self.send_serial_cmd, 10)
    	self.get_logger().info("Initializing Serial communication")
    	self.subscription # prevent unused variable warning

        # Creating String publisher (Publishing the status)
    	self.publisher = self.create_publisher(String,'/string_topic',10)

    	# Timer callback for publisher
    	self.time_period = 10  # seconds 
    	self.timer = self.create_timer(self.time_period, self.read_serial)

   	def send_serial_cmd(self,msg):

        # Relay board toogles to Docking light if the message is true.
        if msg.data == True:
            self.get_logger().info("Initiating docking lights")

           	# Before  toggling the relay, one of the output should be active 
	    	# Activating navigation light 
	    	self.set_active1 =  [cmd_activate, address_relay, Nav_light, Nav_single]
	    	self.single_array = bytearray(self.set.active1)

	    	# Sending serial signal 
	    	self.send_serial(self.single_array)  

	    	# Creating a byte array for docking light 
            self.toggle1 = [cmd_toggle, address_relay, Dock_light, Dock_XOR]
            self.bytearray_1 = bytearray(self.toggle1)

            # Sending the signal
            self.get_logger().info("Sending serial : 8-1-00000001-XOR")
            self.send_serial(self.bytearray_1)

       	else :
           	self.get_logger().info("Initiating Navigation lights")

            # Before  toggling the relay, one of the output should be active 
	    	# Activating docking lights 
	    	self.set_active2 =  [cmd_activate, address_relay, Dock_light, Dock_single]
	    	self.single_array2 = bytearray(self.set.active2)
	
	    	# Sending serial signal 
	    	self.send_serial(self.single_array2) 

	    	# Creating a byte array for docking light 
            self.toggle2 = [cmd_toggle, address_relay, Nav_light,Nav_XOR]
            self.bytearray_2 = bytearray(self.toggle2) 

            # Sending the signal
            self.get_logger().info("Sending serial : 8-1-00000010-XOR")
            self.send_serial(self.bytearray_2)
   
    def send_serial(self, send):
        self.ser.write(send)
		print('Serial sent to the relay board ')

    def read_serial(self):

		# Expected response from relay board
		expected_response_single = 249
		expected_response_toggle = 247            # Expected byte 0 response for toggle
        
		# Read serial from the relay board
        self.response_serial = self.ser.read(1)
		
		# Decoding the response command
		self.decode= int.from_bytes(self.response_serial)
        print('Response : ' + self.decode)

		if  self.decode  == expected_response_single:
			pass

        elif self.decode == expected_response_toggle:
			self.pub_status()       # publishing the status

       	else :
            self.pub_status()       # publishing the status
	    	rclpy.shutdown()

    def pub_status(self):
       	# Defining message type for publisher 
        status = String()

      	# Condition for publisher 
        if self.decode == 255:   # if serial has working properly
			 status.data = "Lights turned on."
            self.get_logger().info("Relay board : Lights turned on")
        	self.publisher.publish(status)    # Publishing status to the same topic 

        else :                               # if serial has error 
            status.data = "Failed connection with relay board"
            self.get_logger().error("Error : Failed connection")

def main(args=None):
    rclpy.init(args=args)
    topic_subscriber = Subscriber()
    rclpy.spin(topic_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


