#!/usr/bin/env python3

import math

from detectors.base_detector_algorithm import BasePathAlgorithm

from models.pallete_perception import PalletDetectionResult
from models.pallete_path import PalletPath, Waypoint


class PalletPathAlgorithm(
    BasePathAlgorithm[
        PalletDetectionResult,
        PalletPath,
    ]
):
    """
    Generates a path along the centerline of the pallet,
    from one end of the pallet to the other end.
    """

    def __init__(
        self,
        pallet_length: float = 1.2,
        waypoint_spacing: float = 0.25,
    ):
        self.pallet_length = pallet_length
        self.waypoint_spacing = waypoint_spacing

    def generate(
        self,
        data: PalletDetectionResult,
    ) -> PalletPath:

        # ---------------------------------------------------------
        # Validate detection
        # ---------------------------------------------------------

        if not data.detected:
            return PalletPath(
                valid=False,
                reason="Pallet not detected",
            )

        if data.centroid is None:
            return PalletPath(
                valid=False,
                reason="Pallet centroid is unavailable",
            )

        if self.pallet_length <= 0.0:
            return PalletPath(
                valid=False,
                reason="Pallet length must be greater than zero",
            )

        if self.waypoint_spacing <= 0.0:
            return PalletPath(
                valid=False,
                reason="Waypoint spacing must be greater than zero",
            )

        # ---------------------------------------------------------
        # Pallet center
        # ---------------------------------------------------------

        cx = float(data.centroid[0])
        cy = float(data.centroid[1])

        # ---------------------------------------------------------
        # Pallet longitudinal orientation
        # ---------------------------------------------------------

        yaw = float(data.orientation)

        direction_x = math.cos(yaw)
        direction_y = math.sin(yaw)

        # ---------------------------------------------------------
        # Pallet start/end
        # ---------------------------------------------------------

        half_length = self.pallet_length / 2.0

        extension = self.waypoint_spacing

        start_x = (
            cx
            - direction_x * (half_length + extension)
        )

        start_y = (
            cy
            - direction_y * (half_length + extension)
        )

        end_x = (
            cx
            + direction_x * (half_length + extension)
        )

        end_y = (
            cy
            + direction_y * (half_length + extension)
        )

        # ---------------------------------------------------------
        # Generate centerline
        # ---------------------------------------------------------
        path_length = self.pallet_length + (2.0 * extension)
        waypoints = []

        distance = 0.0

        while distance <= path_length:

            ratio = distance / path_length

            x = start_x + (
                end_x - start_x
            ) * ratio

            y = start_y + (
                end_y - start_y
            ) * ratio

            waypoints.append(
                Waypoint(
                    x=x,
                    y=y,
                    yaw=yaw,
                )
            )

            distance += self.waypoint_spacing

        # ---------------------------------------------------------
        # Make sure exact end point is included
        # ---------------------------------------------------------

        if not waypoints or (
            abs(
                waypoints[-1].x - end_x
            ) > 1e-6
            or
            abs(
                waypoints[-1].y - end_y
            ) > 1e-6
        ):
            waypoints.append(
                Waypoint(
                    x=end_x,
                    y=end_y,
                    yaw=yaw,
                )
            )

        # ---------------------------------------------------------
        # Validate
        # ---------------------------------------------------------

        if len(waypoints) < 2:
            return PalletPath(
                valid=False,
                reason="Unable to generate pallet centerline",
            )

        return PalletPath(
            waypoints=waypoints,
            valid=True,
            reason="Pallet start-to-end path generated",
        )