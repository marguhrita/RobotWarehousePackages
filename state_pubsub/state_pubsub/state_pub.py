import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import IntEnum


class RobotState(IntEnum):
    ONLINE = 0
    OFFLINE = 1
    NAVIGATING = 2
    NAV_DONE = 3
    NAV_ERROR = 4
    PINGING = 5



class RobotStatePub(Node):

    def __init__(self):
        super().__init__('robot_state_pub')

        #parameters
        #Default values
        self.declare_parameter('namespace', 'default')
        self.declare_parameter('on_robot', True) 

        #Get Actual parameters
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        on_robot : bool = self.get_parameter('on_robot').get_parameter_value().bool_value


        self.publisher_ = self.create_publisher(String, 'robots_state', 10)
        timer_period = 5  # seconds

        # Start online timer only if on robot
        if on_robot:
            self.timer = self.create_timer(timer_period, lambda: self.publish(self.namespace))


    def publish(self, name : str, state = RobotState.ONLINE):
        msg = String()

        # Remove the forward slash...
        if name.startswith("/"):
            name = name[1:]

        msg.data = f"{name}:{state}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    robot_state_pub = RobotStatePub()

    rclpy.spin(robot_state_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()