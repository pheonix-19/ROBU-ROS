import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from warehouse_msgs.msg import Task
from std_msgs.msg import String

class TaskClient(Node):
    def __init__(self, ns='robot1'):
        super().__init__('task_client')
        self.ns = ns
        self.nav_client = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        # For demo: subscribe to a simple topic to receive a Task
        self.create_subscription(Task, f'/{ns}/task', self.on_task, 10)
        self.feedback_pub = self.create_publisher(String, f'/{ns}/task_feedback', 10)
        self.get_logger().info(f'TaskClient ready in namespace: {ns}')

    def on_task(self, task: Task):
        self.get_logger().info(f'Received task {task.id}, navigating to pickup...')
        self.navigate(task.pickup_pose, after=lambda: self.navigate(task.dropoff_pose, after=self.done))

    def navigate(self, pose: PoseStamped, after=None):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda fut: self._on_goal_sent(fut, after))

    def _on_goal_sent(self, fut, after):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda rf: self._on_result(rf, after))

    def _on_result(self, rf, after):
        status = rf.result().result
        self.get_logger().info('Arrived at goal (status ok)')
        msg = String()
        msg.data = 'arrived'
        self.feedback_pub.publish(msg)
        if after:
            after()

    def done(self):
        self.get_logger().info('Task complete')
        m = String(); m.data = 'done'
        self.feedback_pub.publish(m)

def main():
    rclpy.init()
    import sys
    ns = sys.argv[1] if len(sys.argv) > 1 else 'robot1'
    node = TaskClient(ns=ns)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
