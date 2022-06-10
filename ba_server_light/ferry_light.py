import serial 
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class Subscriber(Node):

    # Defining the serial port connected to relay board 
    #Serial_port = '/dev/ttyS0' ####### yet to be confirmed 

    #byte0
    # Command for the relay board 
    cmd_initial = b'00000001'  # initializing the relay board 
    cmd_toggle =  b'00000008'  # toggle the light 

    # byte1
    # Address of the relay board
    address_relay = b'00000001' # Only one relay board 

    #byte2
    # Defining the data to the relay board 
    Dock_light = b'00000001'   # First relay output for docking light
    Nav_light = b'00000010'    # Second relay output for navigation light

    #byte3
    # XOR sum 
    #Dock_XOR = cmd_toggle^address_relay^Dock_light # XOR for docking light output
    #Nav_XOR =  cmd_toggle^address_relay^Nav_light  # XOR for navigation light output

    def __init__(self):

        # Node for the Ferry light
        super().__init__('light_subscriber')

        ''' # Checking the serial port 
        if not os.path.exists(self.Serial_port):
            self.get_logger().error("Serial port does not exists :" + self.Serial_port + "command not sent to the relay board")
            rclpy.shutdown()'''

        # Start the serial communication
        '''self.ser = serial.Serial(
            port=self.Serial_port, 
            baudrate=19200, 
            parity= serial.PARITY_NONE, 
            stopbits = serial.STOPBITS_ONE, 
            bytesize= serial.EIGHTBITS,
            timeout=1)   # Serial data configuration based on Conrad manual 
        
        # Checking the serial port is open 
        if(ser.isOpen()==False):
            ser.open() '''     # Opens the serial port 

        # Initializing relay board 
        #command = 
        #self.initial = send_serial() 

        # Creating Boolean subscription 
        self.subscription = self.create_subscription(Bool,'/topic', self.send_serial_cmd, 10)
        self.get_logger().info("Serial communication started")

        # Creating String publisher (Publishing the status)
        self.publisher = self.create_publisher(String,'/topic',10)

    def send_serial_cmd(self,msg):
        
        # Relay board toogles to Docking light if the message is true.
        if msg.data == True:
            self.get_logger().info("Initiating docking lights")
            # serial code
            # Creating a byte array for docking light 
            self.toggle1 = [cmd_toggle, address_relay, Dock_light,Dock_XOR]
            self.bytearray_1 = bytearray(self.toggle1)

            # Sending the signal
            self.get_logger().info("Sending serial : 8-1-00000001-XOR")
            self.send_serial(self.bytearray_1)
            
            # Response from the relay board
            if(self.ser.readline() ==5):
                self.get_logger().error("Error : Something went wrong in the relay board!!")
                status = "Error in the relay board"
                self.publisher.publish(status)    # Publishing status to the same topic 
                rclpy.shutdown()

            else : 
                response = self.ser.readline()    # response in bytes 
                print("Response : ",response)
                response[0] = b'11110111'         # command response for toggle is 247 
                status = "Successfully turned docking lights on."
                self.publisher.publish(status)    # Publishing status to the same topic 
                self.get_logger().info("Docking lights turned on")

        else :
            # serial code 
            self.get_logger().info("Initiating Navigation lights")
            # serial code
            # Creating a byte array for docking light 
            self.toggle2 = [cmd_toggle, address_relay, Nav_light,Nav_XOR]
            self.bytearray_2 = bytearray(self.toggle2)

            # Sending the signal
            self.get_logger().info("Sending serial : 8-1-00000010-XOR")
            self.send_serial(self.bytearray_2)
            
            # Response from the relay board
            if(self.ser.readline() ==5):
                self.get_logger().error("Error : Something went wrong in the relay board!!")
                status = "Error in the relay board"
                self.publisher.publish(status)    # Publishing status to the same topic 
                rclpy.shutdown()

            else : 
                response = self.ser.readline()    # response in bytes 
                print("Response : ",response)
                response[0] = b'11110111'         # command response for toggle is 247 
                status = "Successfully turned navigation lights on."
                self.publisher.publish(status)    # Publishing status to the same topic 
                self.get_logger().info("Navigation lights turned on")
    
    def send_serial(self, send):
        self.ser.write(send)

def main(args=None):
    rclpy.init(args=args)
    topic_subscriber = Subscriber()
    rclpy.spin(topic_subscriber)
    rclpy.shutdown()

    print('Hi from ba_server_light.')


if __name__ == '__main__':
    main()
