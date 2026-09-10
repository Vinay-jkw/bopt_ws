#!/usr/bin/env python3

from typing import List

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from models.cluster import Cluster

from visualization.base_visualizer import BaseVisualizer
from visualization.markers_factory import MarkerFactory
from visualization.colors import Colors


class MarkerVisualizer(BaseVisualizer):
    """
    Creates RViz markers for PPTS visualization.
    """

    def __init__(
        self,
        frame_id: str = "load_wheel_base_link",
        namespace: str = "ppts",
    ):
        super().__init__(frame_id, namespace)

    # ==========================================================
    # PUBLIC API
    # ==========================================================

    def create_cluster_points(
        self,
        clusters: List[Cluster],
    ) -> MarkerArray:

        marker_array = MarkerArray()

        for cluster in clusters:

            marker_array.markers.append(
                self._cluster_points(cluster)
            )

        return marker_array

    def create_bounding_boxes(
        self,
        clusters: List[Cluster],
    ) -> MarkerArray:

        marker_array = MarkerArray()

        for cluster in clusters:

            marker_array.markers.append(
                self._bounding_box(cluster)
            )

        return marker_array

    def create_cluster_ids(
        self,
        clusters: List[Cluster],
    ) -> MarkerArray:

        marker_array = MarkerArray()

        for cluster in clusters:

            marker_array.markers.append(
                self._cluster_text(cluster)
            )

        return marker_array

    # ==========================================================
    # PRIVATE
    # ==========================================================

    def _cluster_points(
        self,
        cluster: Cluster,
    ) -> Marker:

        marker = MarkerFactory.create(
            marker_type=Marker.POINTS,
            marker_id=cluster.id,
            frame_id=cluster.cloud.frame_id,
            namespace="cluster_points",
        )

        marker.scale.x = 0.03
        marker.scale.y = 0.03

        marker.color = Colors.cluster_color(cluster.id)

        for x, y in cluster.cloud.points:

            p = Point()

            p.x = float(x)
            p.y = float(y)
            p.z = 0.0

            marker.points.append(p)

        return marker

    def _bounding_box(
        self,
        cluster: Cluster,
    ) -> Marker:

        pts = cluster.cloud.points

        min_x = pts[:, 0].min()
        max_x = pts[:, 0].max()

        min_y = pts[:, 1].min()
        max_y = pts[:, 1].max()

        marker = MarkerFactory.create(
            marker_type=Marker.LINE_STRIP,
            marker_id=cluster.id + 1000,
            frame_id=cluster.cloud.frame_id,
            namespace="cluster_bbox",
        )

        marker.scale.x = 0.02

        marker.color = Colors.WHITE

        corners = [

            (min_x, min_y),

            (max_x, min_y),

            (max_x, max_y),

            (min_x, max_y),

            (min_x, min_y)

        ]

        for x, y in corners:

            p = Point()

            p.x = float(x)
            p.y = float(y)
            p.z = 0.0

            marker.points.append(p)

        return marker

    def _cluster_text(
        self,
        cluster: Cluster,
    ) -> Marker:

        marker = MarkerFactory.create(
            marker_type=Marker.TEXT_VIEW_FACING,
            marker_id=cluster.id + 2000,
            frame_id=cluster.cloud.frame_id,
            namespace="cluster_text",
        )

        marker.scale.z = 0.15

        marker.color = Colors.WHITE

        marker.pose.position.x = float(
            cluster.cloud.points[:, 0].mean()
        )

        marker.pose.position.y = float(
            cluster.cloud.points[:, 1].mean()
        )

        marker.pose.position.z = 0.15

        marker.text = str(cluster.id)

        return marker