import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from enum import IntEnum
from dataclasses import dataclass


class RobotState(IntEnum):
    ONLINE = 0
    OFFLINE = 1
    NAVIGATING = 2
    NAV_DONE = 3
    NAV_ERROR = 4
    PINGING = 5


@dataclass
class BotEntry:
    name: str
    state: RobotState
    nav_manager : object = None
    initial_pose : tuple[float,float,float] = None
    delivery_pose : tuple[float,float,float] = None
    battery_state : float = 0

class RobotStateSub(Node):

    def __init__(self):
        super().__init__('robot_state_sub')
        self.state_sub = self.create_subscription(
            String,
            'robots_state',
            self.state_listener_callback,
            10)
        
        self.battery_state_subs = []
        self.battery_state_set = {}

        # Initialize bots
        self.bots : list[BotEntry] = []
        self.robot_names = set()

    def state_listener_callback(self, msg):
        namespace, state= msg.data.split(":")
        # Add robot to bots if not already there
        # Using sets for efficiency

        state = int(state)
        # If the robot does not exist yet
        if namespace not in self.robot_names:
            self.robot_names.add(namespace)
            self.bots.append(BotEntry(namespace, state))

            # Create battery state subscription for that robot
            self.battery_state_subs.append(self.create_subscription(
            BatteryState,
            f"{namespace}/battery_state",
            lambda msg: self.battery_state_callback(msg, namespace),
            10)
            )

            self.battery_state_set[namespace] = False

        # Handle navigation states
        elif state == 2 or state == 3 or state == 4:
            b = self.search_bots(namespace)
            b.state = RobotState(state)

        else:
            b = self.search_bots(namespace)
            if b.state == RobotState.PINGING:
            # Reset to online
                b.state = RobotState(0)
                #self.get_logger().info(f"Reset bot {namespace} to {b.state.name}")



            #self.get_logger().info('state: "%s"' % state)

    def battery_state_callback(self, msg : BatteryState, namespace : str):
        if not self.battery_state_set[namespace]:
            b = self.search_bots(namespace)
            b.battery_state = msg.percentage
            #self.get_logger().info('Sent a battery state!')
            self.battery_state_set[namespace] = True


    def search_bots(self, name : str) -> BotEntry:
        bot : BotEntry
        for b in self.bots:
            if b.name == name:
                return b
            
        print(f"Bot {name} not found!")
        return None
        
        

def main(args=None):
    rclpy.init(args=args)

    robot_state_sub = RobotStateSub()

    rclpy.spin(robot_state_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()