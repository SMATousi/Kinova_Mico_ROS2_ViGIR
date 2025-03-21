#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Time
from kinova_msgs.action import ArmPose
from std_msgs.msg import String
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import time

# import tf_transformations

openvla_prediciton = []  # Ensure this is initialized appropriately.
current_pose = None

class ManualPublisher(Node):
    def __init__(self):
        super().__init__('manual_publisher')
        self.publisher_ = self.create_publisher(String, '/f_ing_tcp', 1)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/openvla_prediction',
            self.listener_callback,
            1
        )

    def listener_callback(self, msg):
        global openvla_prediciton
        # self.get_logger().info(f'I heard openvla_prediction: {msg.data}')
        openvla_prediciton = msg.data



class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/m1n6s200_driver/out/tool_pose',
            self.listener_callback,
            1
        )

    def listener_callback(self, msg):
        global current_pose
        # self.get_logger().info(f'I heard tool_pose: {msg}')
        current_pose = msg.pose
        


        

class CartesianPoseClient(Node):
    def __init__(self):
        super().__init__('cartesian_pose_client')

        # It is important to check that `current_pose` and `openvla_prediciton`
        # have been set before using them. Here we assume they are set.
        global current_pose, openvla_prediciton
        if current_pose is None or not openvla_prediciton:
            self.get_logger().error("Current pose or openvla_prediciton not set!")
            # Depending on your application, you might want to wait or exit.
            return
        self.get_logger().info(f'The deltas are = {openvla_prediciton}')
        self.position = [current_pose.position.x - openvla_prediciton[0],
                         current_pose.position.y - openvla_prediciton[1],
                         current_pose.position.z - openvla_prediciton[2]]

        self.orientation = [current_pose.orientation.x - openvla_prediciton[0],
                            current_pose.orientation.y - openvla_prediciton[1],
                            current_pose.orientation.z - openvla_prediciton[2],
                            current_pose.orientation.w - openvla_prediciton[3]]

        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

    def send_goal(self, current_time):
        # Convert current_time to a message
        time_msg = current_time.to_msg()
        t = Time()
        t.sec = time_msg.sec
        t.nanosec = time_msg.nanosec

        goal = ArmPose.Goal()
        goal.pose.header = Header(frame_id='m1n6s200_link_base')
        goal.pose.header.stamp = t
        goal.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        goal.pose.pose.orientation = Quaternion(x=self.orientation[0], y=self.orientation[1], 
                                                  z=self.orientation[2], w=self.orientation[3])

        self.get_logger().info('Sending goal...')
        self.client.send_goal_async(goal)
        self.get_logger().info('Goal sent, waiting for result...')
        # You may add result callbacks if needed.



# class CartesianPoseClient(Node):
#     def __init__(self):
#         super().__init__('cartesian_pose_client')

#         self.position = [current_pose.position.x + openvla_prediciton[0], 
#                          current_pose.position.y + openvla_prediciton[1], 
#                          current_pose.position.z + openvla_prediciton[2]]

#         # delta_quaternion = tf_transformations.quaternion_from_euler(openvla_prediciton[3], 
#         #                                                             openvla_prediciton[4], 
#         #                                                             openvla_prediciton[5])

#         self.orientation = [current_pose.orientation.x + openvla_prediciton[0], 
#                             current_pose.orientation.y + openvla_prediciton[1], 
#                             current_pose.orientation.z + openvla_prediciton[2], 
#                             current_pose.orientation.w + openvla_prediciton[3]]

#         self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')

#         self.get_logger().info('Waiting for action server...')
#         self.client.wait_for_server()

#     def send_goal(self, current_time):

#         time_msg = current_time.to_msg()

#         t = Time()
#         t.sec = time_msg.sec
#         t.nanosec = time_msg.nanosec

#         goal = ArmPose.Goal()
#         goal.pose.header = Header(frame_id='m1n6s200_link_base')
#         goal.pose.header.stamp = t
#         goal.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
#         goal.pose.pose.orientation = Quaternion(x=self.orientation[0], y=self.orientation[1], z=self.orientation[2], w=self.orientation[3])

#         # print(goal)
#         self.get_logger().info('Sending goal...')
#         self.client.send_goal_async(goal)

#         self.get_logger().info('Waiting for result...')

#         # self.client.wait_for_result()

#         # result = self.client.get_result()
#         # if not result:
#         #     self.get_logger().info('Goal failed')
#         # else:
#         #     self.get_logger().info(f'Result: {result.pose.pose}')

def main(args=None):
    rclpy.init(args=args)
    # Create nodes
    open_vla_node = MinimalSubscriber()
    position_node = PositionSubscriber()
    publisher = ManualPublisher()
    message = 'done'
    publisher.publish_message(message)
    time.sleep(5)
    

    # Use a MultiThreadedExecutor to spin multiple nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(open_vla_node)
    executor.add_node(position_node)

    # Start executor in a separate thread if needed
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # rate = open_vla_node.create_rate(2)

    # Wait for the subscribers to receive messages
    # (Adjust sleep duration as needed or use a more robust synchronization method)
    
    client_node = CartesianPoseClient()

    while (1):
        client_node = CartesianPoseClient()
        current_time = client_node.get_clock().now()
        client_node.send_goal(current_time)
        time.sleep(1)
        client_node.destroy_node()
        publisher.publish_message(message)
        time.sleep(5)

    


    open_vla_node.destroy_node()
    position_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
