from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SafetyPublisher(Node):
    """Auxiliary node that owns the safety and status publishers.

    WorkflowHandler borrows publisher_ from this node so it can publish
    safety field commands without creating duplicate topic advertisements.
    """

    def __init__(self):
        super().__init__('Task_allocator')
        self.publisher_ = self.create_publisher(String, '/byd/safety', 10)
        self.safety_switch_publisher = self.create_publisher(
            String, '/byd/safety', 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_publisher = self.create_publisher(
            String, '/byd/current_task', 10
        )
        self.state_publisher = self.create_publisher(String, '/byd/status', 10)
        print("publishers_created")
