#!/usr/bin/env python3

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray

from models.roi import ROI
from visualization.base_visualizer import BaseVisualizer


class ROIVisualizer(BaseVisualizer):
    """
    Visualize pallet ROIs as rectangular outlines with labels.
    """

    def __init__(
        self,
        frame_id: str = "load_wheel_base_link",
        namespace: str = "rois",
    ):
        super().__init__(frame_id, namespace)

    def create_markers(
        self,
        rois: list[ROI],
    ) -> MarkerArray:

        marker_array = MarkerArray()

        marker_id = 0

        for index, roi in enumerate(rois):

            # =====================================================
            # ROI Rectangle
            # =====================================================

            rectangle = self.create_marker(
                marker_id=marker_id,
                marker_type=Marker.LINE_STRIP,
            )

            rectangle.scale.x = 0.02

            rectangle.color = self.create_color(
                0.0,
                1.0,
                0.0,
                1.0,
            )

            rectangle.lifetime = Duration()

            rectangle.points = [

                self.create_point(roi.xmin, roi.ymin),
                self.create_point(roi.xmax, roi.ymin),
                self.create_point(roi.xmax, roi.ymax),
                self.create_point(roi.xmin, roi.ymax),
                self.create_point(roi.xmin, roi.ymin),

            ]

            marker_array.markers.append(rectangle)

            marker_id += 1

            # =====================================================
            # ROI Label
            # =====================================================

            text = self.create_marker(
                marker_id=marker_id,
                marker_type=Marker.TEXT_VIEW_FACING,
                namespace=f"{self.namespace}_text",
            )

            text.pose.position = self.create_point(
                (roi.xmin + roi.xmax) / 2.0,
                (roi.ymin + roi.ymax) / 2.0,
                0.05,
            )

            text.scale.z = 0.10

            text.color = self.create_color(
                1.0,
                1.0,
                1.0,
                1.0,
            )

            text.text = f"ROI {index + 1}"

            text.lifetime = Duration()

            marker_array.markers.append(text)

            marker_id += 1

        return marker_array