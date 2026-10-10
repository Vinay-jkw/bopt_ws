
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

        ns = self.get_namespace()
        self.tf_prefix = ns[1:] + '/' if ns != '/' else ''

        self.marker_publisher = self.create_publisher(
            Marker, 'visualization_marker_pap', 10
        )
        self.boolean_publisher = self.create_publisher(
            Bool, 'pap_field_status', 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_subscription = self.create_subscription(
            LaserScan, 'Lidar_RFT', self.scan_callback, 10
        )

        self.timer = self.create_timer(
            0.5, self.publish_quadrilateral
        )

        # Existing PAP detection dimensions.
        self.base = 0.15
        self.location_width = 0.96
        self.location_length = 0.45 + self.base

        # Detection polygon stays in the LiDAR frame.
        self.lidar_polygon = [
            Point(x=self.base, y=-self.location_width / 2, z=0.0),
            Point(x=self.location_length,
                  y=-self.location_width / 2, z=0.0),
            Point(x=self.location_length,
                  y=self.location_width / 2, z=0.0),
            Point(x=self.base, y=self.location_width / 2, z=0.0),
        ]

    def point_in_quadrilateral(self, x, y):
        """Check whether a LiDAR-frame point is inside the polygon."""
        polygon = self.lidar_polygon
        n = len(polygon)

        if n < 3:
            return False

        inside = False
        j = n - 1

        for i in range(n):
            xi, yi = polygon[i].x, polygon[i].y
            xj, yj = polygon[j].x, polygon[j].y

            if (yi > y) != (yj > y):
                x_intersection = (
                    (xj - xi) * (y - yi) / (yj - yi) + xi
                )
                if x < x_intersection:
                    inside = not inside

            j = i

        return inside

    def scan_callback(self, scan_msg):
        """Publish whether at least two valid scan points are inside."""
        expected_frame = (
            self.tf_prefix + 'front_lidar_frame_right'
        )

        # Do not compare coordinates from different frames.
        if (
            scan_msg.header.frame_id
            and scan_msg.header.frame_id != expected_frame
        ):
            self.boolean_publisher.publish(Bool(data=False))
            return

        points_inside = 0
        threshold = 2

        # Preserve the existing alternate-point sampling.
        for i in range(0, len(scan_msg.ranges), 2):
            r = scan_msg.ranges[i]

            # Ignore invalid and out-of-range measurements.
            if not math.isfinite(r):
                continue
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            angle = (
                scan_msg.angle_min + i * scan_msg.angle_increment
            )
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if self.point_in_quadrilateral(x, y):
                points_inside += 1
                if points_inside >= threshold:
                    break

        self.boolean_publisher.publish(
            Bool(data=(points_inside >= threshold))
        )

    def publish_quadrilateral(self):
        """Transform the polygon for visualization only."""
        source_frame = (
            self.tf_prefix + 'front_lidar_frame_right'
        )
        target_frame = self.tf_prefix + 'base_link'

        try:
            if not self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            ):
                return

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )

            marker = Marker()
            marker.header.frame_id = target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'quadrilateral'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add the first point again to close the visualization.
            source_points = (
                self.lidar_polygon + [self.lidar_polygon[0]]
            )
            transformed_points = []

            for point in source_points:
                point_stamped = PointStamped(
                    header=Header(frame_id=source_frame),
                    point=point,
                )
                transformed = tf2_geometry_msgs.do_transform_point(
                    point_stamped, transform
                )
                transformed_points.append(transformed.point)

            marker.points = transformed_points
            self.marker_publisher.publish(marker)

        except Exception as exc:
            self.get_logger().warn(
                f'Could not transform quadrilateral points: {exc}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = QuadrilateralPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
