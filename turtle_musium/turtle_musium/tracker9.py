import rclpy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration 
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import numpy as np
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

class tracker_node(Node):
    def __init__(self):
        super().__init__('tracker_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.close_enough_distance = 1.0

        self.create_subscription(PointStamped,'/robot9/point_camera',self.callback_depth,10)

        self.pub_stay = self.create_publisher(Bool,'/robot9/gift_stay',10)
        # self.sub_gift_start = self.create_subscription(Bool,'/robot9/giftshop',self.callback_giftshop,10)
        self.sub_label = self.create_subscription(String,"robot9/painting",self.callback_label,10)
        self.latest_map_point = None
        self.goal_handle = None
        self.block_goal_updates = True
        self.close_distance_hit_count = 0
        self.last_feedback_log_time = 0
        self.pose = PoseStamped()
        self.gift_put = False
        self.label = None


        # self.create_timer(0.5, self.process_frame)


    def callback_label(self,msg: String):
        self.label = msg.data

            
    def callback_depth(self,pt):

        try:
            if self.label in ("pingu","moo","haowl","pinga"):
                pt.point.z = pt.point.z - 0.3
            else:
                pt.point.z = pt.point.z - 0.5


            self.get_logger().info(f"Detected at map: (x = {pt.point.x:.2f}, y = {pt.point.y:.2f} z = {pt.point.z:.2f})")
            if pt.point.z <= 0.0:
                pt.point.z = 0.0
            pt_map = self.tf_buffer.transform(pt, 'map', timeout=Duration(seconds=0.5))
            self.latest_map_point = pt_map

            # Don't send more goals if we're already close
            if self.block_goal_updates:
                self.get_logger().info(f"Within ({self.close_enough_distance}) meter — skipping further goal updates.")
                

            self.get_logger().info(f"Detected at map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f},{pt_map.point.y:.2f})")

            if self.goal_handle:
                self.get_logger().info("Canceling previous goal...")
                self.goal_handle.cancel_goal_async()

            self.send_goal()
            



        except Exception as e:
            self.get_logger().warn(f"TF transform to map failed: {e}")



    def send_goal(self):

        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.x = self.latest_map_point.point.x
        self.pose.pose.position.y = self.latest_map_point.point.y
        self.pose.pose.orientation.z = 180.0
        # self.pose.pose.orientation.w = 1

        # pose.pose.position.x = -2.0
        # pose.pose.position.y = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = self.pose

        self.get_logger().info(f"Sending goal to: ({self.pose.pose.position.x:.2f}, {self.pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        # Require 3 close readings to trigger the lock
        if self.current_distance is not None and self.current_distance < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
            self.block_goal_updates = True
            self.get_logger().info("Confirmed: within 1 meter — blocking further goal updates.")

        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {self.current_distance:.2f} m")
            self.last_feedback_log_time = now

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        if self.label in ("pingu","moo","haowl","pinga"):
            msg = Bool()
            msg.data = True
            self.pub_stay.publish(msg)
            self.get_logger().info("pub_True")

    #     self._get_result_future = self.goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.goal_result_callback)

    # def goal_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Goal finished with result code: {future.result().status}")
    #     if self.gift_put:
    #         self.pub_stay.publish(Bool(data=True))
    #         self.get_logger().info("pub_True")
    #         self.gift_put = False

    #     self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = tracker_node()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
        # rclpy.spin(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()