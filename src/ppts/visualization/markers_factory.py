#!/usr/bin/env python3

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker


class MarkerFactory:
    """
    Factory for creating RViz markers with common defaults.

    Every marker in PPTS should be created using this factory.
    """

    @staticmethod
    def create(
        marker_type: int,
        marker_id: int,
        frame_id: str,
        namespace: str,
    ) -> Marker:

        marker = Marker()

        # Header
        marker.header.frame_id = frame_id

        # Namespace
        marker.ns = namespace

        # Identification
        marker.id = marker_id

        # Marker Type
        marker.type = marker_type

        # Action
        marker.action = Marker.ADD

        # Orientation
        marker.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0,
        )

        # Infinite lifetime
        marker.lifetime = Duration(
            sec=0,
            nanosec=0,
        )

        return marker