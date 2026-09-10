#!/usr/bin/env python3

from abc import ABC

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class BaseVisualizer(ABC):
    """
    Base class for all PPTS visualizers.

    Provides common helper functions shared across
    PointCloud and Marker visualizers.
    """

    def __init__(
        self,
        frame_id: str,
        namespace: str = "ppts",
    ):
        self.frame_id = frame_id
        self.namespace = namespace

    # ---------------------------------------------------------
    # Header
    # ---------------------------------------------------------

    def create_header(
        self,
        frame_id: str = None,
    ) -> Header:
        """
        Create a ROS message header.

        Parameters
        ----------
        frame_id : str, optional
            Override default frame.

        Returns
        -------
        Header
        """

        header = Header()

        header.frame_id = frame_id or self.frame_id

        return header
    def create_marker(
        self,
        marker_id: int,
        marker_type: int,
        namespace: str = None,
        ) -> Marker:

            marker = Marker()

            marker.header = self.create_header()

            marker.ns = namespace or self.namespace

            marker.id = marker_id

            marker.type = marker_type

            marker.action = Marker.ADD

            return marker
    # ---------------------------------------------------------
    # Geometry
    # ---------------------------------------------------------

    @staticmethod
    def create_point(
        x: float,
        y: float,
        z: float = 0.0,
    ) -> Point:
        """
        Create a geometry_msgs/Point.
        """

        return Point(
            x=float(x),
            y=float(y),
            z=float(z),
        )

    # ---------------------------------------------------------
    # Colors
    # ---------------------------------------------------------

    @staticmethod
    def create_color(
        r: float,
        g: float,
        b: float,
        a: float = 1.0,
    ) -> ColorRGBA:
        """
        Create a ColorRGBA message.
        """

        return ColorRGBA(
            r=float(r),
            g=float(g),
            b=float(b),
            a=float(a),
        )