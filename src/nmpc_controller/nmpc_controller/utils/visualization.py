# Handles all RViz2 publishing for the controller. Kept separate so visualization
# code does not clutter the control logic in controller.py.

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class Visualizer:
    def __init__(self, node):
        self._node = node
        self._path_pub = node.create_publisher(Path, "/visualization_path", 10)
        self._marker_pub = node.create_publisher(Marker, "/target_point_marker", 10)

    def publish_path(self, path):
        # Called once after the path is loaded. Shows the full planned route in RViz2.
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        for pt in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)

    def publish_target_marker(self, target):
        # Called every control tick. The red sphere moves along the path showing
        # the current lookahead point the controller is steering toward.
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        self._marker_pub.publish(m)
