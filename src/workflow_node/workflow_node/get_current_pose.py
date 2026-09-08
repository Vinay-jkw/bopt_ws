from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy,QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
import rclpy

class CurrentLocationNode(Node):
    def __init__(self):
        super().__init__('current_location_node')
        qos_settings = QoSProfile(depth=10)
        qos_settings.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.current_pose_callback, qos_settings)
        self.current_pose = None
        self.success = False

    def current_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose

    
def main():
    rclpy.init()
    node = CurrentLocationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_pose is not None:
            break
    current_pose = node.current_pose

    current_formatted_pose = [current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w]
    print('fetched current pose:',current_formatted_pose)
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == "__main__":
  
    main()