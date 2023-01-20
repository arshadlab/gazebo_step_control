import sys
from threading import Thread
from gazebo_step_control_interface.srv import StepControl
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
import time

class SteppingController(Node):

    def __init__(self):
        super().__init__('stepping_client')
        self.step_enable_service = self.create_client(SetBool, 'step_control_enable')
        while not self.step_enable_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('step_control_enable service not available, waiting again...')
        
        self.step_control_service = self.create_client(StepControl, 'step')
        while not self.step_control_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('step control service not available, waiting again...')
    
        
    def run(self):
        enable_req = SetBool.Request()
        enable_req.data = True        
        self.step_enable_service.call(enable_req)
        while True:
             step_req = StepControl.Request()
             step_req.steps = 1
             step_req.block = True       
             self.step_control_service.call(step_req)          

def main(args=None):
    rclpy.init(args=args)
    stepping_client = SteppingController()
    spin_thread = Thread(target=rclpy.spin, args=(stepping_client,))
    spin_thread.start()
    stepping_client.run()
    stepping_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()