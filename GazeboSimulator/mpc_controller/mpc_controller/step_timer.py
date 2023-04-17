import rclpy
from numpy import sin,cos, pi, array, linalg, sqrt
from rclpy.node import Node
from std_msgs.msg import Int32

class StepTimerNode(Node):

    def __init__(self):
        super().__init__('step_timer')
        self.start_timer = False

def main(args=None):
    rclpy.init(args=args)

    step_timer = StepTimerNode()

    rclpy.spin(step_timer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    step_timer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()