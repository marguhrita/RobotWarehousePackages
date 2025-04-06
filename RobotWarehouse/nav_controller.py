from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import subprocess
import time, os
from util import RobotStatePublisher, RobotState


class NavManager():
    def __init__(self : object, initial_position : tuple[float, float, float], delivery_position : tuple[float,float,float], namespace : str, publisher : RobotStatePublisher):

        self.namespace = namespace
        self.delivery_pose = delivery_position
        self.initial_pose = initial_position

        # Start nav
        self.nav2_start()
      
        print(f"namespace:{self.namespace}")
        
        self.pub = publisher

    def set_initial_pose(self, pose : tuple[float,float,float]):

        goal_pose = f"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, pose: {{pose: {{position: {{x: {pose[0]}, y: {pose[1]}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: -0.005862246656604559, w: 0.9999828168844388}}}}}}}}"
        command = ["ros2", "topic", "pub", "--once", "--qos-reliability", "reliable", f"/{self.namespace}/initialpose", "geometry_msgs/PoseWithCovarianceStamped", goal_pose]
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def nav2_exit(self):
        self.process.kill()

    def nav2_start(self):
        print(f"Launching nav for robot: {self.namespace}")

        map_dir = os.path.abspath("map.yaml")

        self.process = subprocess.Popen(
            ['ros2', 'launch', "robot_w", "multi_robot.launch.py", f"namespace:={self.namespace}", f"map:={map_dir}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.set_initial_pose(self.initial_pose)

    def navigate_to_start_pose(self):
        self.navigate_to_position(self.initial_pose[0], self.initial_pose[1], self.initial_pose[2])

    def navigate_to_delivery_pose(self):
        self.navigate_to_position(self.delivery_pose[0], self.delivery_pose[1], self.delivery_pose[2])

    def fetch_item(self, x : float, y : float, z : float, wait_time : float = 4):
        self.navigate_to_position(x, y, z)
        time.sleep(wait_time)
        self.navigate_to_delivery_pose()
        time.sleep(wait_time)
        self.navigate_to_start_pose()

    def navigate_to_position(self, x,y,z):
        print(f"namespace:{self.namespace}")

        goal_pose = f"pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation:{{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}}}}"

        command = ["ros2", "action", "send_goal",
                    f"/{self.namespace}/navigate_to_pose",
                    "nav2_msgs/action/NavigateToPose",
                    goal_pose]

        self.nav_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        for line in self.nav_process.stdout:
            if line.strip().startswith("Goal accepted"):
                self.pub.publish(self.namespace, 2)
            elif line.strip().startswith("Goal finished"):
                self.pub.publish(self.namespace, 3)
            elif line.strip().startswith("Goal Aborted"):
                self.pub.publish(self.namespace, 4)
                self.navigate_to_start_pose()

            print("Output:", line.strip())

            
        
            
            
