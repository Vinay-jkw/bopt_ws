import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Bool

class QuadrilateralPublisher(Node):
    def __init__(self):
        super().__init__('quadrilateral_publisher')

        # Publishers for PAP quadrilateral
        self.marker_publisher_pap = self.create_publisher(Marker, '/visualization_marker_pap', 10)
        self.boolean_publisher_pap = self.create_publisher(Bool, '/pap_field_status', 10)

        # Publishers for DS quadrilateral
        self.marker_publisher_ds = self.create_publisher(Marker, '/visualization_marker_ds', 10)
        self.boolean_publisher_ds = self.create_publisher(Bool, '/ds_field_status', 10)
        
        # Set up the TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_subscription = self.create_subscription(LaserScan, '/Lidar_RFT', self.scan_callback, 10)

        # Create a timer to periodically publish the markers
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_quadrilaterals)

        # To store transformed quadrilateral points
        self.transformed_points_pap = []
        self.transformed_points_ds = []

    def point_in_quadrilateral(self, x, y, transformed_points):
        """Check if a point is inside the quadrilateral using the ray-casting algorithm."""
        n = len(transformed_points)
        inside = False

        p1x, p1y = transformed_points[0].x, transformed_points[0].y
        for i in range(n):
            p2x, p2y = transformed_points[i].x, transformed_points[i].y

            if (y > min(p1y, p2y)):
                if (y <= max(p1y, p2y)):
                    if (x <= max(p1x, p2x)):
                        if (p1y != p2y):
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def scan_callback(self, scan_msg):
        """Callback function for LaserScan messages."""
        # Check points for PAP quadrilateral
        points_inside_pap = self.count_points_inside(scan_msg, self.transformed_points_pap)
        # Publish PAP status
        self.publish_boolean(self.boolean_publisher_pap, points_inside_pap)

        # Check points for DS quadrilateral
        points_inside_ds = self.count_points_inside(scan_msg, self.transformed_points_ds)
        # Publish DS status
        self.publish_boolean(self.boolean_publisher_ds, points_inside_ds)

    def count_points_inside(self, scan_msg, transformed_points):
        """Count the number of points inside a given quadrilateral."""
        points_inside = 0  # Counter for points inside the quadrilateral
        threshold = 2      # Threshold for publishing True

        for i in range(0, len(scan_msg.ranges), 2):
            range_value = scan_msg.ranges[i]
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = range_value * math.cos(angle)
            y = range_value * math.sin(angle)

            try:
                if self.point_in_quadrilateral(x, y, transformed_points):
                    points_inside += 1
                    if points_inside >= threshold:
                        break
            except Exception as e:
                self.get_logger().warn(f'Error checking point: {e}')

        return points_inside

    def publish_boolean(self, publisher, points_inside, threshold=2):
        """Publish True if points_inside >= threshold, else False."""
        msg = Bool(data=(points_inside >= threshold))
        publisher.publish(msg)

    def publish_quadrilaterals(self):
        """Publish both PAP and DS quadrilaterals."""
        self.publish_quadrilateral_pap()
        self.publish_quadrilateral_ds()

    def publish_quadrilateral_pap(self):
        """Publish the PAP quadrilateral marker."""
        try:
            # Lookup the transformation
            transform = self.tf_buffer.lookup_transform('base_link', 'front_lidar_frame_right', rclpy.time.Time())

            # Create and configure the Marker message for PAP
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "quadrilateral_pap"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Define PAP quadrilateral parameters
            base = 0.42
            location_width = 0.7
            location_length = 0.45 + base

            # Define the 4 corners of the PAP quadrilateral in load_wheel_base_link frame
            points = [
                Point(x=base, y=-location_width/2, z=0.0),  # Bottom-left
                Point(x=location_length, y=-location_width/2, z=0.0),  # Top-left
                Point(x=location_length, y=location_width/2, z=0.0),  # Top-right
                Point(x=base, y=location_width/2, z=0.0),  # Bottom-right
                Point(x=base, y=-location_width/2, z=0.0)  # Close the loop
            ]

            # Transform the points to Lidar_RFT frame
            transformed_points = self.transform_points(points, 'front_lidar_frame_right', transform)
            self.transformed_points_pap = transformed_points
            marker.points = transformed_points

            # Publish the PAP marker
            self.marker_publisher_pap.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform PAP quadrilateral points: {e}')

    def publish_quadrilateral_ds(self):
        """Publish the DS quadrilateral marker."""
        try:
            # Lookup the transformation
            transform = self.tf_buffer.lookup_transform('base_link', 'front_lidar_frame_right', rclpy.time.Time())

            # Create and configure the Marker message for DS
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "quadrilateral_ds"
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Define DS quadrilateral parameters
            base = 0.35
            location_width = 0.7
            location_length = 0.45 + base

            # Define the 4 corners of the DS quadrilateral in base_link frame
            points = [
                Point(x=base, y=-location_width/2, z=0.0),  # Bottom-left
                Point(x=location_length, y=-location_width/2, z=0.0),  # Top-left
                Point(x=location_length, y=location_width/2, z=0.0),  # Top-right
                Point(x=base, y=location_width/2, z=0.0),  # Bottom-right
                Point(x=base, y=-location_width/2, z=0.0)  # Close the loop
            ]

            # Transform the points to Lidar_RFT frame
            transformed_points = self.transform_points(points, 'base_link', transform)
            self.transformed_points_ds = transformed_points
            marker.points = transformed_points

            # Publish the DS marker
            self.marker_publisher_ds.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform DS quadrilateral points: {e}')

    def transform_points(self, points, source_frame, transform):
        """Transform a list of points from the source frame to the target frame."""
        transformed_points = []
        for point in points:
            point_stamped = PointStamped(
                header=Header(frame_id=source_frame),
                point=point
            )
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            transformed_points.append(transformed_point_stamped.point)
        return transformed_points

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    quadrilateral_publisher = QuadrilateralPublisher()

    # Spin the node to keep it active
    rclpy.spin(quadrilateral_publisher)

    # Shutdown the ROS 2 node
    quadrilateral_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
