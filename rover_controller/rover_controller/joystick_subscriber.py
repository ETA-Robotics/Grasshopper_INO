#!/usr/bin/env python3
import rclpy
import serial
import time
import struct
from rclpy.node import Node
from sensor_msgs.msg import Joy
try:
    ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening or using serial port: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")


class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__("joystick_subscriber")
        self.joy1_subscription = self.create_subscription(
            Joy, '/joy1', self.joy1_callback, 10)
        self.joy2_subscription = self.create_subscription(
            Joy, '/joy2', self.joy2_callback, 10)      

        self.get_logger().info("Joystick Subscriber has been started 3.1")

    def joy1_callback(self, msg):
        # Assuming throttle is axis 2 on X52 controller
        throttle = msg.axes[2]
        throttle = round(throttle, 3)
        #print("throttle:", throttle)
        stopButton = msg.buttons[7]
        CalibrationButton = msg.buttons[30]
     
        #print("Throttles position:", throttle)
        data = f"{throttle}"

        if (CalibrationButton == 1):
            data = f"{'4.0'}"

        if(stopButton == 1):     
            data = f"{'2.0'}"

      
        
        self.send_packet('B', data)  # Packet type B
        # Delay for 50 milliseconds
        time.sleep(0.05)


    def joy2_callback(self, msg):
        # Assuming throttle is axis 2 on Yoke
        angle = msg.axes[0]
        angle = round(angle, 3)
        #print("Yoke angle:", angle)
        data = f"{angle}"
        self.send_packet('C', data)  # Packet type C
        # Delay for 50 milliseconds
        time.sleep(0.05)

    def send_packet(self, packet_id, data):
        message = f"{packet_id},{data}\n"  
        ser.write(message.encode())


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()