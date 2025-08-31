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
import time

SERVICE_NAME = '/detected_cam/select'
USE_ROBOT = True

class Robot9ToMain(Node):
    def __init__(self):
        super().__init__('robot9_to_main')

        if USE_ROBOT:
            self.navigator = TurtleBot4Navigator()
            self.navigator.undock()
            initial_pose = self.navigator.getPoseStamped([-0.23, 0.28], TurtleBot4Directions.SOUTH) # undock pose 
            self.navigator.setInitialPose(initial_pose)

            self.get_logger().info("set initial pose")

            self.navigator.waitUntilNav2Active()
            self.get_logger().info("NAV2")
            self.gift_result = False
            self.patrol = True
            self.gift_stay = False
            self.goal_options = {
                    'home': self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH),

                    'entrance': self.navigator.getPoseStamped([-1.97, 5.25], TurtleBot4Directions.SOUTH),
                    'painting_1': self.navigator.getPoseStamped([-0.23, 4.67], TurtleBot4Directions.NORTH),
                    'painting_2': self.navigator.getPoseStamped([-1.71, 3.93], TurtleBot4Directions.SOUTH),
                    'painting_3': self.navigator.getPoseStamped([-0.98, 2.06], TurtleBot4Directions.NORTH),
                    'exit': self.navigator.getPoseStamped([-2.7, 1.62], TurtleBot4Directions.WEST),
                    'gift_shop': self.navigator.getPoseStamped([-1.72, -0.09], TurtleBot4Directions.SOUTH),
                    'gift_stay': self.navigator.getPoseStamped([-1.37, 0.61], TurtleBot4Directions.WEST),
                }
            # self.goal_options = {
            #     'home': {'pose': [0.0, 0.0], 'direction': TurtleBot4Directions.SOUTH},
            #     'entrance': {'pose': [-1.97, 5.25], 'direction': TurtleBot4Directions.SOUTH},
            #     'painting_1': {'pose': [-0.23, 4.67], 'direction': TurtleBot4Directions.NORTH},
            #     'painting_2': {'pose': [-1.71, 3.93], 'direction': TurtleBot4Directions.SOUTH},
            #     'painting_3': {'pose': [-0.98, 2.06], 'direction': TurtleBot4Directions.NORTH},
            #     'exit': {'pose': [-2.7, 1.62], 'direction': TurtleBot4Directions.WEST},
            #     'gift_shop': {'pose': [-1.72, -0.09], 'direction': TurtleBot4Directions.SOUTH},
            #     'gift_stay': {'pose': [-1.37, 0.61], 'direction': TurtleBot4Directions.WEST},
            #     'Exit': None
            # }
            # self.get_robot_position = self.create_subscription(
            #     PoseWithCovarianceStamped, '/robot8/amcl_pose', self.cb_robot_position, 10
            # )
        self.sub_person_exit = self.create_subscription(Bool,'/robot9/person_exit',self.callback_start,10)
        self.sub_gift = self.create_subscription(Bool, '/robot9/gift', self.callback_gift_result, 10)
        self.sub_gfit_stay = self.create_subscription(Bool,'/robot9/gift_stay',self.callback_gift_stay,10)
        self.sub_guide_end = self.create_subscription(Bool,'/robot9/guide_done',self.callback_guide_end,10)
        
        self.pub_gift_start = self.create_publisher(Bool,'/robot9/gift_start',10)
        self.pub_person = self.create_publisher(Bool,'/robot9/person',10)


    # giftshop 출발
    def callback_gift_result(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("gift shop go")
            self.gift_result = True
            self.patrol = False
    # 기념품 가져오고 대기
    def callback_gift_stay(self,msg: Bool):
        if msg:
            self.go_to_pose_and_check(self.goal_options['gift_stay'])
            self.gift_stay = True
    # 가이드로봇 가이드 끝        
    def callback_guide_end(self,msg: Bool):
        if msg and self.gift_stay:
            self.pub_person.publish(msg)
            self.gift_stay = False

    def callback_start(self, msg: String):
        person = Bool()
        person.data = False
        self.pub_person.publish(person)

        self.navigator.cancelTask()

        if self.patrol:
            self.get_logger().info("경로 시작")

            sequence = ['entrance', 'painting_1', 'painting_2', 'painting_3', 'exit']

            for location in sequence:
                if self.gift_result:
                    self.get_logger().warn("중단 감지. 행동 중단 및 종료.")
                    self.navigator.cancelTask()
                    self.perform_interrupt_action()
                    return

                result = self.go_to_pose_and_check(location)
                if result:
                    self.perform_interrupt_action()
                    return
                # if location not in ('exit','entrance'): 
                #     if not self.call_check_painting_service():
                #         self.get_logger().info("그림 없음, 경로 종료.")
                #         return


    def go_to_pose_and_check(self, location_name):
        self.get_logger().info(f"{location_name} 로 이동")

        self.navigator.startToPose(self.goal_options[location_name])

        while not self.navigator.isTaskComplete():
            if self.gift_result:
                self.get_logger().warn("이동 중 인터럽트 발생 → 경로 취소")
                self.navigator.cancelTask()
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"{location_name} 도착")
        return False

    def call_check_painting_service(self):
        self.get_logger().info("그림 유무 확인 중...")

        client = self.create_client(Trigger, '/robot9/painting')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('서비스 서버에 연결할 수 없습니다.')
            return False

        req = Trigger.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info("그림 있음: 다음 장소로 이동")
                return True
            else:
                self.get_logger().info("그림 없음: 이동 중단")
                return False
        else:
            self.get_logger().error("서비스 응답 없음")
            return False
    
    def perform_interrupt_action(self):
        self.gift_result = False
        self.go_to_pose_and_check(self.goal_options['gift_shop'])

        msg = Bool()
        msg.data = True
        self.pub_gift_start.publish(msg)
    def callback_gift_stay(self,msg= Bool):
        if msg:
            self.go_to_pose_and_check(self.goal_options['gift_stay'])
            self.gift_stay = True

            



    

        
def main(args=None):
    rclpy.init(args=args)
    node = Robot9ToMain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()