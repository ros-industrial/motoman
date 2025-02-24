import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class FollowJointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('follow_joint_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')
        # self.joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
        self.joint_names = ["group_1/joint_1", "group_1/joint_2", "group_1/joint_3", "group_1/joint_4", "group_1/joint_5", "group_1/joint_6"]
        self.current_joint_states = None
        self.current_joint_states = [0.8498216635719337,-0.07335297278307744, -1.6250996380076779, 0.04404431972481594, 0.6207489574138463, 0.6973037196961867]


        # self.subscription = self.create_subscription(
        #     JointState, '/joint_states', self.joint_state_callback, 10)
        # self.subscription  # prevent unused variable warning

    # def joint_state_callback(self, msg):
    #     print("getting callback")
    #     self.current_joint_states = msg

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        if self.current_joint_states is None:
            self.get_logger().error('No joint states received!')
            return

        # if set(self.joint_names) != set(self.current_joint_states.name):
        #     self.get_logger().fatal("Mismatch between expected and received joint names!")
        #     return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.joint_names
        
        q0 = self.current_joint_states #.position
        q1 = list(q0)
        q2 = list(q0)
        q1[-1] -= math.radians(20)
        qdot = [0.0] * len(self.joint_names)
        
        goal_msg.trajectory.points.append(JointTrajectoryPoint(
            positions=q0, velocities=qdot, time_from_start=Duration(sec=0)))
        goal_msg.trajectory.points.append(JointTrajectoryPoint(
            positions=q1, velocities=qdot, time_from_start=Duration(sec=5)))
        goal_msg.trajectory.points.append(JointTrajectoryPoint(
            positions=q2, velocities=qdot, time_from_start=Duration(sec=10)))

        self.get_logger().info('Sending goal...')
        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result.error_code}')
        rclpy.shutdown()


def main():
    rclpy.init()
    client = FollowJointTrajectoryClient()
    rclpy.spin_once(client, timeout_sec=2.0)  # Ensure at least one joint state message is received
    client.send_goal()
    rclpy.spin(client)
    #client.destroy_node()

if __name__ == '__main__':
    main()
