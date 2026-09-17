#!/usr/bin/env python3

import math

from typing import List

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from models.pallete_path import PalletPath, Waypoint

from visualization.base_visualizer import BaseVisualizer
from visualization.markers_factory import MarkerFactory
from visualization.colors import Colors


class PalletPathVisualizer(BaseVisualizer):
    """
    Visualizes the generated pallet approach path.

    The visualizer creates ROS MarkerArray messages containing:
        - pallet
        - pallet center
        - approach path
        - waypoints
        - waypoint orientation
    """

    def __init__(
        self,
        frame_id: str = "load_wheel_base_link",
        namespace: str = "ppts",
        waypoint_size: float = 0.08,
        pallet_size_x: float = 1.4,
        pallet_size_y: float = 0.8,
    ):
        super().__init__(frame_id, namespace)

        self.waypoint_size = waypoint_size
        self.pallet_size_x = pallet_size_x
        self.pallet_size_y = pallet_size_y

    def create_markers(
        self,
        path: PalletPath,
        pallet_centroid,
        pallet_orientation: float,
    ) -> MarkerArray:

        marker_array = MarkerArray()

        if not path.valid:
            return marker_array

        if pallet_centroid is None:
            return marker_array

        cx = float(pallet_centroid[0])
        cy = float(pallet_centroid[1])

        # ---------------------------------------------------------
        # Pallet
        # ---------------------------------------------------------

        marker_array.markers.append(
            self._create_pallet_marker(
                cx,
                cy,
                pallet_orientation,
            )
        )

        # ---------------------------------------------------------
        # Pallet center
        # ---------------------------------------------------------

        marker_array.markers.append(
            self._create_center_marker(
                cx,
                cy,
            )
        )

        # ---------------------------------------------------------
        # Waypoints
        # ---------------------------------------------------------

        for index, waypoint in enumerate(path.waypoints):

            marker_array.markers.append(
                self._create_waypoint_marker(
                    waypoint,
                    index,
                )
            )

            marker_array.markers.append(
                self._create_waypoint_arrow(
                    waypoint,
                    index,
                )
            )

        # ---------------------------------------------------------
        # Path
        # ---------------------------------------------------------

        if len(path.waypoints) > 1:

            marker_array.markers.append(
                self._create_path_marker(
                    path.waypoints,
                )
            )

        return marker_array

    # =========================================================
    # PALLET
    # =========================================================

    def _create_pallet_marker(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Marker:

        marker = MarkerFactory.create(
            Marker.CUBE,
            0,
            self.frame_id,
            self.namespace,
        )

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05

        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.scale.x = self.pallet_size_x
        marker.scale.y = self.pallet_size_y
        marker.scale.z = 0.1

        marker.color = Colors.BLUE

        return marker

    # =========================================================
    # PALLET CENTER
    # =========================================================

    def _create_center_marker(
        self,
        x: float,
        y: float,
    ) -> Marker:

        marker = MarkerFactory.create(
            Marker.SPHERE,
            1,
            self.frame_id,
            self.namespace,
        )

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color = Colors.RED

        return marker

    # =========================================================
    # WAYPOINT
    # =========================================================

    def _create_waypoint_marker(
        self,
        waypoint: Waypoint,
        index: int,
    ) -> Marker:

        marker = MarkerFactory.create(
            Marker.SPHERE,
            100 + index,
            self.frame_id,
            self.namespace,
        )

        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.position.z = 0.08

        marker.scale.x = self.waypoint_size
        marker.scale.y = self.waypoint_size
        marker.scale.z = self.waypoint_size

        marker.color = Colors.GREEN

        return marker

    # =========================================================
    # WAYPOINT ORIENTATION
    # =========================================================

    def _create_waypoint_arrow(
        self,
        waypoint: Waypoint,
        index: int,
    ) -> Marker:

        marker = MarkerFactory.create(
            Marker.ARROW,
            200 + index,
            self.frame_id,
            self.namespace,
        )

        start = Point()

        start.x = waypoint.x
        start.y = waypoint.y
        start.z = 0.10

        arrow_length = 0.30

        end = Point()

        end.x = (
            waypoint.x
            + arrow_length * math.cos(waypoint.yaw)
        )

        end.y = (
            waypoint.y
            + arrow_length * math.sin(waypoint.yaw)
        )

        end.z = 0.10

        marker.points.append(start)
        marker.points.append(end)

        marker.scale.x = 0.04
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color = Colors.YELLOW

        return marker

    # =========================================================
    # PATH
    # =========================================================

    def _create_path_marker(
        self,
        waypoints: List[Waypoint],
    ) -> Marker:

        marker = MarkerFactory.create(
            Marker.LINE_STRIP,
            300,
            self.frame_id,
            self.namespace,
        )

        marker.scale.x = 0.04

        marker.color = Colors.CYAN

        for waypoint in waypoints:

            point = Point()

            point.x = waypoint.x
            point.y = waypoint.y
            point.z = 0.03

            marker.points.append(point)

        return marker