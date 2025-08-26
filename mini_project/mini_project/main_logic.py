import rclpy
from std_msgs.msg import String, Bool

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        self.navigator = TurtleBot4Navigator()

        # Start on dock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before intialising pose')
            self.navigator.dock()

        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        # Undock
        self.navigator.undock()

        # Prepare goal pose options
        self.goal_options = {
            'home': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH),
            'living_room': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.EAST),
            'bed_room': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH),
            'kitchen': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH),
            'Exit': None
        }

        self.navigator.info('Welcome to the mail delivery service.')

        ## webcams request 처리

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
    def on_service_pose(self, msg: String):
        # 여기서 이미지를 파일로 저장/전처리/전송 등 필요 작업 수행
        self.where_car = msg
        # if self.where_car == "living_room":
        #     self.navigator.startToPose(self.goal_options[1]['pose'])
        # elif self.where_car == "kitchen":
        #     self.navigator.startToPose(self.goal_options[1]['pose'])
        self.navigator.startToPose(self.goal_options[self.where_car])

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
