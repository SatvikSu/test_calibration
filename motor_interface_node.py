#!/usr/bin/env python3
"""
motor_interface_node.py
- Converts to encoder counts (counts = 250 * cm).
- Runs synchronized wheel & move
- Publishes wheel position, velocity, leg angle and speed (u) command magnitude.
- Subscribes to motor control output ONLY (speed and leg angle)

PUB: motor/velocity
PUB: leg/angle
PUB: init/motor_ready
SUB: init/motor_init
SUB: motor/speed_cmd
"""

import time
import threading
from math import copysign
from math import pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Int32MultiArray, Float32MultiArray



class MotorInterfaceNode(Node):
    def __init__(self):

        super().__init__('motor_interface_node')
        self.ready_pub = self.create_publisher(String, 'init/motor_ready', 10)
        self.calibration_sub = self.create_subscription(String, 'init/motor_init', self.calibration, 10)
        
    # ---------- CALIBRATION SEQUENCE -----------

        # Calibration method runs through each leg motor and runs a calibration sequence for to find min/max angles
    def calibration(self, msg:String):

        if (msg.data == "No calibration" ):
            print("Motor interface node is ready.")
            # Publish "Ready" signal since no calibration
            ready_msg = String()
            ready_msg.data = "Ready"
            self.ready_pub.publish(ready_msg)
            self.initialization()
            return

        if 'L_FL' in msg.data:
            self.stall_detect('L_FL')
        if 'L_FR' in msg.data:
            self.stall_detect('L_FR')
        if 'L_RL' in msg.data:
            self.stall_detect('L_RL')
        if 'L_RR' in msg.data:
            self.stall_detect('L_RR')    
        print("All legs finished calibrating")
        
        # Publish "Ready" signal when ready line is reached
        ready_msg = String()
        ready_msg.data = "Ready"
        self.ready_pub.publish(ready_msg)


    # Dummy stall detection function
    def stall_detect(self, leg_name):
        print(f'Finding min of {leg_name}')
        time.sleep(1)
        print(f'Finding max of {leg_name}')
        time.sleep(1)
        print(f'{leg_name} calibrated')

def main(args=None):
    rclpy.init(args=args)
    node = MotorInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

