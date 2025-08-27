import rclpy
from std_msgs.msg import String, Bool

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SERVICE_NAME = '/detected_cam/select'

import numpy as np

# living_room
# pose:
#   pose:
#     position:
#       x: -2.093113013607228
#       y: -0.2427662848680473
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.5593111127454344
#       w: 0.8289578271296217

# kitchen
# pose:
#   pose:
#     position:
#       x: -2.0943655737323317
#       y: 0.5887783839951957
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.7431899068627759
#       w: 0.6690805350160013

# bed_room
# pose:
#   pose:
#     position:
#       x: -0.5927699176721698
#       y: 2.2097128810830426
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.5780426432258742
#       w: 0.8160065579469595

        # # TEST
        # goal_pose = []
        # goal_pose.append(self.goal_options['living_room'])
        # # goal_pose.append(self.goal_options['kitchen'])
        # # goal_pose.append(self.goal_options['bed_room'])
        # goal_pose.append(self.goal_options['home'])
        # self.navigator.startFollowWaypoints(goal_pose)

        # self.navigator.dock()

USE_ROBOT = False

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        if USE_ROBOT:
            self.navigator = TurtleBot4Navigator()

            # Start on dock
            if not self.navigator.getDockedStatus():
                self.navigator.info('Docking before intialising pose')
                self.navigator.dock()

            # Set initial pose
            initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH) # undock pose 
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info("set initial pose")

            # Wait for Nav2
            self.navigator.waitUntilNav2Active()
            self.get_logger().info("NAV2")

            # Undock
            # self.navigator.undock()

            # Prepare goal pose options
            self.goal_options = {
                'home': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH),
                # 'living_room': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.EAST),
                'living_room': self.navigator.getPoseStamped([-2.09, -0.24], np.rad2deg(0.56)),
                # 'living_room': self.navigator.getPoseStamped([-2.09, -0.24], TurtleBot4Directions.WEST),
                'kitchen': self.navigator.getPoseStamped([-2.09, 0.58], np.rad2deg(0.74)),
                'bed_room': self.navigator.getPoseStamped([-0.59, 2.21], np.rad2deg(-0.57)),
                'Exit': None
            }


        where_car = "None"
        self.cli = self.create_client(Trigger, SERVICE_NAME)
        self.get_logger().info(f'Waiting for service: {SERVICE_NAME}')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Service {SERVICE_NAME} not available')

        while not where_car == "None":
            where_car = self.call_once()
            self.get_logger().info(f"처음 서비스 응답: {where_car}")

        if USE_ROBOT:
            self.navigator.startToPose(self.goal_options[where_car])
        ## Tracking Topic Pub 처리
        self.tracking_publisher = self.create_publisher(String, '/robot8/tracking_object', 10) # QoS default

        self.subscription = self.create_subscription(
            Bool, '/robot8/is_done_track', self.cb_track_done, 10)

        self.sub = self.create_subscription(
            String, '/robot8/which_hand', self.cb_which_hand, 10
        )


    def call_once(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            raise TimeoutError('Service call timed out')
        resp = future.result()
        # resp.success: bool, resp.message: string ("cam0|cam1|none ..." 형식)
        where_car = resp.message
        self.get_logger().info(f'Service response: success={resp.success}, message="{where_car}"')
        return where_car

    ## Tracking 완료 되었나?
    def cb_track_done(self, msg: Bool):
        if msg: # done: True
            pass

    ## 손모양 인식이 들어왔나?
    def cb_which_hand(self, msg: String):
        if msg == "one": # 물병 위치로 (kitchen)
            if USE_ROBOT:
                self.navigator.startToPose(self.goal_options["kitchen"])
            self.tracking_publisher.publish('bottle')
            pass
        elif msg == "five":
            if USE_ROBOT:
                self.navigator.startToPose(self.goal_options["home"])
            self.tracking_publisher.publish('None')
            ### finish logic
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
