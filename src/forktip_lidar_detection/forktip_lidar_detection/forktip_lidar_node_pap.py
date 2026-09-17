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

        # Create a publisher for the Marker message
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker_pap', 10)

        # Create a publisher for the boolean result (True if inside, False if not)
        self.boolean_publisher = self.create_publisher(Bool, '/pap_field_status', 10)
        
        # Set up the TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_subscription = self.create_subscription(LaserScan, '/Lidar_RFT', self.scan_callback, 10)

        # Create a timer to periodically publish the marker
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_quadrilateral)

         # To store transformed quadrilateral points
        self.transformed_points = []

    def point_in_quadrilateral(self, x, y):
        """Check if a point is inside the quadrilateral using the ray-casting algorithm."""
        n = len(self.transformed_points)
        inside = False
        # print(point.point.x)
        # x, y = point.point.x, point.point.y

        p1x, p1y = self.transformed_points[0].x, self.transformed_points[0].y
        for i in range(n):
            p2x, p2y = self.transformed_points[i].x, self.transformed_points[i].y

            # Check if the point is on an edge or within the bounds of the quadrilateral
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
        points_inside = 0  # Counter for points inside the quadrilateral
        threshold = 2  # Set your threshold for how many points should be inside to publish True

        # Iterate through every other range in the LaserScan message
        for i in range(0, len(scan_msg.ranges), 2):
            range_value = scan_msg.ranges[i]

            # Calculate the angle for this scan point
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Calculate the point's (x, y) position in the laser's frame
            x = range_value * math.cos(angle)
            y = range_value * math.sin(angle)

            try:
                # Check if the point lies inside the quadrilateral
                if self.point_in_quadrilateral(x, y):
                    points_inside += 1

                    # Exit loop if threshold is met
                    if points_inside >= threshold:
                        break

            except Exception as e:
                self.get_logger().warn(f'Error transforming point: {e}')

        # Publish True if the number of points inside exceeds the threshold, otherwise False
        if points_inside >= threshold:
            self.boolean_publisher.publish(Bool(data=True))
        else:
            self.boolean_publisher.publish(Bool(data=False))

            

    def publish_quadrilateral(self):
        try:
            # Lookup the transformation between right_forktip_laser and base_link
            if not self.tf_buffer.can_transform(
                'base_link',
                'front_lidar_frame_right',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            ):
                return

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'front_lidar_frame_right',
                rclpy.time.Time()
            )

            # Create a Marker message for the quadrilateral
            marker = Marker()
            marker.header.frame_id = "base_link"  # Now we're visualizing in base_link frame
            marker.header.stamp = self.get_clock().now().to_msg()  # Current time
            marker.ns = "quadrilateral"
            marker.id = 0
            marker.type = Marker.LINE_STRIP  # Define the marker as a line strip
            marker.action = Marker.ADD

            # Set the scale of the marker (line width)
            marker.scale.x = 0.05  # Width of the line

            # Set the color of the marker (RGBA)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha (transparency)

            base = 0.15
            location_width = 0.96
            location_length = 0.45 + base
            

            # Define the 4 corners of the quadrilateral in right_forktip_laser frame
            points = [
                Point(x=base, y=-location_width/2, z=0.0),  # Bottom-left
                Point(x=location_length, y=-location_width/2, z=0.0),  # Top-left
                Point(x=location_length, y=location_width/2, z=0.0),  # Top-right
                Point(x=base, y=location_width/2, z=0.0),  # Bottom-right
                Point(x=base, y=-location_width/2, z=0.0)  # Close the loop by connecting to the first point
            ]
            # Log the original points before transformation
            # self.get_logger().info("Original Points (right_forktip_laser frame):")
            # for point in points:
            #     self.get_logger().info(f"Point: x={point.x}, y={point.y}, z={point.z}")

            # Transform the points to base_link frame
            transformed_points = []
            for point in points:
                # Create a geometry_msgs/PointStamped from the point
                point_stamped = PointStamped(
                    header=Header(frame_id='front_lidar_frame_right'),
                    point=point
                )
                # Transform the point
                transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                
                # Append the transformed point to the list
                transformed_points.append(transformed_point_stamped.point)

            # Log the transformed points after transformation
            # self.get_logger().info("Transformed Points (base_link frame):")
            # for point in transformed_points:
            #     self.get_logger().info(f"Point: x={point.x}, y={point.y}, z={point.z}")

            # Store the transformed points for later use
            self.transformed_points = transformed_points

            # Add the transformed points to the marker
            marker.points = transformed_points

            # Publish the transformed marker
            self.marker_publisher.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform quadrilateral points: {e}')

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
