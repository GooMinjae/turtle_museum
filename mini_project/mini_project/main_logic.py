import rclpy
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



# class MainNode(Node):
#     def __init__(self):
#         super().__init__('main_node')
#         # === (필요 시 수정) 토픽/서비스 이름 ===
#         self.handoff_topics = ['/handoff/cam0', '/handoff/cam1'] # yolo_dual_cam_node와 일치 필요
#         self.service_name = '/yolo_dual_cam/push_once_and_exit'
#         # handoff는 transient local이므로, 구독 측에서도 transient local로 두면 늦게 붙어도 수신 가능
#         qos_handoff = QoSProfile(depth=1)
#         qos_handoff.reliability = ReliabilityPolicy.RELIABLE
#         qos_handoff.history = HistoryPolicy.KEEP_LAST
#         qos_handoff.durability = DurabilityPolicy.TRANSIENT_LOCAL
#         self.bridge = CvBridge()
#         self.received = [False, False]
#         # handoff 이미지 구독 (2개)
#         self.subs = []
#         for i, topic in enumerate(self.handoff_topics):
#             self.subs.append(
#                 self.create_subscription(Image, topic, lambda msg, idx=i: self.on_img(msg, idx), qos_handoff)
# )
#         # 서비스 클라이언트
#         self.cli = self.create_client(Trigger, self.service_name)
#         self.get_logger().info('Waiting for service...')
#         self.cli.wait_for_service()
#         # 서비스 호출: 마지막 프레임 publish 요청
#         self.get_logger().info('Calling push_once_and_exit...')
#         self.req = Trigger.Request()
#         self.future = self.cli.call_async(self.req)
#         self.future.add_done_callback(self.on_service_done)


#     def on_service_done(self, future):
#         try:
#             resp = future.result()
#             self.get_logger().info(f"Service response: success={resp.success}, msg='{resp.message}'")
#         except Exception as e:
#             self.get_logger().error(f'Service call failed: {e}')


#     def on_img(self, msg: Image, idx: int):
#         # 여기서 이미지를 파일로 저장/전처리/전송 등 필요 작업 수행
#         cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         self.get_logger().info(f'Received handoff image from cam{idx} ({cv_img.shape[1]}x{cv_img.shape[0]})')
#         self.received[idx] = True
#         # 두 카메라 모두 수신하면 종료
#         if all(self.received):
#             self.get_logger().info('Received images from both cameras. Exiting main_node...')
#             rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     node = MainNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()
# if __name__ == '__main__':
#     main()

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.0, 0.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-3.0, -3.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([3.0, -3.0], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
