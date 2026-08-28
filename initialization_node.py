import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class InitializationNode(Node):

    def __init__(self):
        super().__init__('initialization_node')
        self.motor_pub = self.create_publisher(String, 'init/motor_init', 10)
        # self.motor_sub = self.create_subscription(String, 'init/motor_ready', self.motor_check, 10)

        # Start with motor initialization
        print("-------- MOTOR INITIALIZATION -------")
        
        # Start a background thread dedicated entirely to reading user input
        self.input_thread = threading.Thread(target=self.read_motor_init, daemon=True)
        self.input_thread.start()

    def read_motor_init(self):
        msg = String()
        init = False
        # This blocks the background thread, but NOT the ROS 2 node
        while not init:
            user_input = input("Calibrate? (y/n): ")    
            if user_input == "y":
                msg.data = "Start"
                init = True
                calibrate = True
            elif user_input == "n":
                msg.data = "No calibration"
                init = True
                calibrate = False

        if calibrate:

            calibrate_l_fl = False
            while not calibrate_l_fl:
                user_input = input("Calibrate L_FL? (y/n): ")
                if user_input == "y":
                    msg.data += " | L_FL"
                    calibrate_l_fl = True
                elif user_input == "n":
                    calibrate_l_fl = True

            calibrate_l_fr = False
            while not calibrate_l_fr:
                user_input = input("Calibrate L_FR? (y/n): ")
                if user_input == "y":
                    msg.data += " | L_FR"
                    calibrate_l_fr = True
                elif user_input == "n":
                    calibrate_l_fr = True

            calibrate_l_rl = False
            while not calibrate_l_rl:
                user_input = input("Calibrate L_RL? (y/n): ")
                if user_input == "y":
                    msg.data += " | L_RL"
                    calibrate_l_rl = True
                elif user_input == "n":
                    calibrate_l_rl = True

            calibrate_l_rr = False
            while not calibrate_l_rr:
                user_input = input("Calibrate L_RR? (y/n): ")
                if user_input == "y":
                    msg.data += " | L_RR"
                    calibrate_l_rr = True
                elif user_input == "n":
                    calibrate_l_rr = True

        self.motor_pub.publish(msg)

    '''
    def motor_check(self, msg: String):
        self.motor_ready = msg.data
        if self.motor_ready == "Ready":
            print("-------- CONTROL INITIALIZATION -------")
            self.input_thread2 = threading.Thread(target=self.read_control_init, daemon=True)
            self.input_thread2.start()

    def read_control_init(self):
        user_input = input("Hit 'Enter' to start control node: ")
        print("ALL NODES INITIALIZED")
    '''

def main(args=None):
    rclpy.init(args=args)
    node = InitializationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()