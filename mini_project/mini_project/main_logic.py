import rclpy
from std_msgs.msg import String, Bool

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        self.navigator = TurtleBot4Navigator()

        # Start on dock
        # if not self.navigator.getDockedStatus():
        #     self.navigator.info('Docking before intialising pose')
        #     self.navigator.dock()

        # Set initial pose
        # initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        # self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        # self.navigator.waitUntilNav2Active()

        # Undock
        # self.navigator.undock()

        # Prepare goal pose options
        self.goal_options = {
            'home': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH),
            # 'living_room': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.EAST),
            'living_room': self.navigator.getPoseStamped([-2.09, -0.24], np.degrees(0.56)),
            'kitchen': self.navigator.getPoseStamped([-2.09, 0.58], np.degrees(0.74)),
            'bed_room': self.navigator.getPoseStamped([-0.59, 2.21], np.degrees(-0.57)),
            'Exit': None
        }

        # TEST
        goal_pose = []
        goal_pose.append(self.goal_options['living_room'])
        goal_pose.append(self.goal_options['kitchen'])
        goal_pose.append(self.goal_options['bed_room'])
        goal_pose.append(self.goal_options['home'])
        self.navigator.startFollowWaypoints(goal_pose)

        # self.navigator.dock()

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

############3333
        # self.navigator.info('Welcome to the mail delivery service.')

        ## webcams request 처리
        self.cli_push_once = self.create_client(
            Trigger, '/yolo_dual_cam/push_once_and_exit'
        )
        if not self.cli_push_once.service_is_ready():
            self.get_logger().warn(
                "[push_once_and_exit] service not ready yet. Will call when available."
            )

        # (예시) 부팅 후 3초 뒤 한 번 호출해보기
        # self.create_timer(3.0, self._once_call_push_and_cancel_timer)

        ## Tracking Topic Pub 처리
        self.publisher = self.create_publisher(String, '/robot8/tracking_object', 10) # QoS default
        timer_period = 0.5  # seconds

        self.subscription = self.create_subscription(
            Bool, '/robot8/is_done_track', self.cb_track_done, 10)

        self.sub = self.create_subscription(
            String, '/robot8/which_hand', self.cb_which_hand, 10
        )

    def on_service_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Service response: success={resp.success}, msg='{resp.message}'")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    ## RC car (사람) 위치 response
    def on_service_pose(self, msg: Trigger):
        # 여기서 이미지를 파일로 저장/전처리/전송 등 필요 작업 수행
        self.where_car = msg
        # if self.where_car == "living_room":
        #     self.navigator.startToPose(self.goal_options[1]['pose'])
        # elif self.where_car == "kitchen":
        #     self.navigator.startToPose(self.goal_options[1]['pose'])
        # self.navigator.startToPose(self.goal_options[self.where_car])
        self.get_logger().info(f"get msg {msg.data}")

    ## Tracking 완료 되었나?
    def cb_track_done(self, msg: Bool):
        if msg: # done: True
            pass

    ## 손모양 인식이 들어왔나?
    def cb_which_hand(self, msg: String):
        if msg == "one": # 물병 위치로 (kitchen)
            self.navigator.startToPose(self.goal_options["kitchen"])
            pass
        elif msg == "five":
            self.navigator.startToPose(self.goal_options["home"])
            self.publisher.publish('None')
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
