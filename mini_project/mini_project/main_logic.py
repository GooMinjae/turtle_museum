import rclpy
from std_msgs.msg import String, Bool

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

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

USE_ROBOT = True

# def quat_to_yaw(q):
#     # q: geometry_msgs/Quaternion
#     # yaw(Z)만 필요할 때의 변환
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        self.x, self.y = 0.0, 0.0
        self.memorize_enabled = False

        if USE_ROBOT:
            self.navigator = TurtleBot4Navigator()

            # # Start on dock
            # if not self.navigator.getDockedStatus():
            #     self.navigator.info('Docking before intialising pose')
            #     self.navigator.dock()

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
                # 'living_room': self.navigator.getPoseStamped([-2.09, -0.24], np.rad2deg(0.56)),
                'living_room': self.navigator.getPoseStamped([-2.09, -0.24], TurtleBot4Directions.WEST),
                # 'kitchen': self.navigator.getPoseStamped([-2.09, 0.58], np.rad2deg(0.74)),
                'kitchen': self.navigator.getPoseStamped([-1.1, 2.1], TurtleBot4Directions.SOUTH),
                # 'bed_room': self.navigator.getPoseStamped([-0.59, 2.21], np.rad2deg(-0.57)),
                'bed_room': self.navigator.getPoseStamped([-0.59, 2.21], TurtleBot4Directions.EAST),
                'Exit': None
            }

            self.get_robot_position = self.create_subscription(
                PoseWithCovarianceStamped, '/robot8/amcl_pose', self.cb_robot_position, 10
            )

        self.subscription = self.create_subscription(
            Bool, '/robot8/is_done_track', self.cb_track_done, 10)

        self.sub = self.create_subscription(
            String, '/robot8/which_hand', self.cb_which_hand, 10
        )


        where_car = "None"
        ## TEST to bottle
        where_car = "bed_room"
        # self.cli = self.create_client(Trigger, SERVICE_NAME)
        # self.get_logger().info(f'Waiting for service: {SERVICE_NAME}')
        # if not self.cli.wait_for_service(timeout_sec=5.0):
        #     raise RuntimeError(f'Service {SERVICE_NAME} not available')

        while where_car == "None":
            where_car = self.call_once()
            self.get_logger().info(f"처음 서비스 응답: {where_car}")

        self.tracking_publisher = self.create_publisher(String, '/robot8/tracking_object', 10) # QoS default
        if USE_ROBOT:
            self.memorize_enabled = True
            self.prev_x = self.x
            self.prev_y = self.y
            self.get_logger().info(f"current_pose {self.prev_x}, {self.prev_y}")
            self.navigator.startToPose(self.goal_options[where_car])
        ## Tracking Topic Pub 처리 'person' 'bottle' 'None'
        self.tracking_publisher.publish(String(data='person'))



    def cb_robot_position(self, msg: PoseWithCovarianceStamped):
        # 최신 메시지 갱신
        self.latest_msg = msg

        # 연속 저장 모드인 경우에만 저장
        if self.memorize_enabled:
            # self._save_from_msg(msg)
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y

        self.memorize_enabled = False


    def call_once(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        self.get_logger().info("CALL_ONCE")
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
        if msg.data: # done: True
            self.get_logger().info("이전 위치로 이동")
            pass

    ## 손모양 인식이 들어왔나?
    def cb_which_hand(self, msg: String):
        if msg.data == "one": # 물병 위치로 (kitchen)
            if USE_ROBOT:
                self.navigator.startToPose(self.goal_options["kitchen"])
            self.tracking_publisher.publish(String(data='bottle'))
            pass
        elif msg.data == "five":
            if USE_ROBOT:
                self.navigator.startToPose(self.goal_options["home"])
            self.tracking_publisher.publish(String(data='None'))
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
