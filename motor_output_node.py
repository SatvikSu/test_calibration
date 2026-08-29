
import time
import threading
from math import copysign
from math import pi

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Int32MultiArray, Float32MultiArray
from motoron import MotoronI2C # Used for motor actuation
import Jetson.GPIO as GPIO
import smbus # System Management Bus


class MotorOutputNode(Node):
    def __init__(self):
        super().__init__('motor_output_node')
        self.speed_cmd_pub = self.create_publisher(Int32MultiArray, 'motor/speed_cmd', 10)
        self.velocity_sub = self.create_subscription(Float32MultiArray, 'motor/velocity', self.velocityPID, 10)

        # track integral of velocity error 
        self.vel_error_integrals = Int32MultiArray()
        self.vel_error_integrals.data = []
        for i in range(8):
            self.vel_error_integrals.data.append(0)
    def velocityPID(self, msg : Float32MultiArray):
        Kp = 8 * pi/180 # Kp going from omega (rad/s) to motor speed command (from -800 to 800). For motor voltage = 12 V
        Ki = 2 * pi/180 # Ki going from omega (rad/s) * time (s) to motor speed command (from -800 to 800). For motor voltage = 12 V
        # reference velocity in rad/s
        reference_vel = [ 
            0, # W_FL
            90 * pi/180, # W_FR
            0, # W_RL
            0, # W_RR
            0, # L_FL
            90 * pi/180, # L_FR
            0, # L_RL
            0, # L_RR
        ]

        vel_errors = []
        for i in range(8):
            vel_errors[i] = reference_vel[i] - msg.data[i]
            self.vel_error_integrals.data[i] += vel_errors[i] * 0.01 # TODO - use time.perf_time() instead of assuming perfect 100 Hz messages

        speeds = Int32MultiArray()
        speeds.data = []
        for i in range(8):
            speeds.data.append(int(Kp * vel_errors[i] + Ki * self.vel_error_integrals.data[i]))

        self.speed_cmd_pub.publish(speeds)        

def main(args=None):
    rclpy.init(args=args)
    node = MotorOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

