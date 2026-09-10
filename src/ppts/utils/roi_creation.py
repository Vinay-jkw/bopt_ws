#!/usr/bin/env python3

from typing import List

from config.config_models import PalletConfig, ROIConfig
from models.roi import ROI


class ROIGenerator:
    """
    Generates Regions of Interest (ROIs) for pallet detection.

    Supported layouts:
        - grid
        - custom

    Grid layout:
        ROIs are automatically generated from rows x columns.

    Custom layout:
        ROIs are explicitly defined using normalized coordinates
        in the range [0.0, 1.0].
    """

    def __init__(
        self,
        pallet_config: PalletConfig,
        roi_config: ROIConfig,
    ):
        self._pallet = pallet_config
        self._roi = roi_config

    def generate(self) -> List[ROI]:

        if self._roi.layout == "grid":
            return self._generate_grid()

        if self._roi.layout == "custom":
            return self._generate_custom()

        raise ValueError(
            f"Unsupported ROI layout: {self._roi.layout}"
        )

    # ---------------------------------------------------------
    # Grid
    # ---------------------------------------------------------

    def _generate_grid(self) -> List[ROI]:

        rows = self._roi.grid.rows
        columns = self._roi.grid.columns

        if rows <= 0 or columns <= 0:
            raise ValueError(
                "ROI rows and columns must be greater than zero"
            )

        pallet_length = self._pallet.geometry.length
        pallet_width = self._pallet.geometry.width

        inset_width = self._roi.geometry.inset_width
        inset_length = self._roi.geometry.inset_length
        lidar_offset = self._roi.geometry.lidar_offset

        usable_length = pallet_length + 2.0 * inset_length
        usable_width = pallet_width + 2.0 * inset_width

        if usable_length <= 0:
            raise ValueError(
                "ROI inset is too large for pallet length"
            )

        if usable_width <= 0:
            raise ValueError(
                "ROI inset is too large for pallet width"
            )

        cell_length = usable_length / rows 
        cell_width = usable_width / columns 

        rois: List[ROI] = []

        roi_id = 1

        for row in range(rows):

            for column in range(columns):

                xmin = (
                    lidar_offset
                    - inset_length/2
                    + row * cell_length
                )

                xmax = xmin + cell_length

                ymin = (
                    -pallet_width / 2.0
                    - inset_width
                    + column * cell_width
                )

                ymax = ymin + cell_width

                rois.append(
                    ROI(
                        id=f"roi_{roi_id}",
                        xmin=xmin,
                        xmax=xmax,
                        ymin=ymin,
                        ymax=ymax,
                    )
                )

                roi_id += 1

        return rois

    # ---------------------------------------------------------
    # Custom
    # ---------------------------------------------------------

    def _generate_custom(self) -> List[ROI]:

        pallet_length = self._pallet.length
        pallet_width = self._pallet.width

        inset = self._roi.inset
        lidar_offset = self._roi.lidar_offset

        rois: List[ROI] = []

        for region in self._roi.regions:

            self._validate_custom_region(region)

            xmin = (
                lidar_offset
                + inset
                + region.xmin * (
                    pallet_length - 2.0 * inset
                )
            )

            xmax = (
                lidar_offset
                + inset
                + region.xmax * (
                    pallet_length - 2.0 * inset
                )
            )

            ymin = (
                -pallet_width / 2.0
                + inset
                + region.ymin * (
                    pallet_width - 2.0 * inset
                )
            )

            ymax = (
                -pallet_width / 2.0
                + inset
                + region.ymax * (
                    pallet_width - 2.0 * inset
                )
            )

            rois.append(
                ROI(
                    id=region.id,
                    xmin=xmin,
                    xmax=xmax,
                    ymin=ymin,
                    ymax=ymax,
                )
            )

        return rois

    # ---------------------------------------------------------
    # Validation
    # ---------------------------------------------------------

    @staticmethod
    def _validate_custom_region(region) -> None:

        values = [
            region.xmin,
            region.xmax,
            region.ymin,
            region.ymax,
        ]

        if any(value < 0.0 or value > 1.0 for value in values):
            raise ValueError(
                f"Custom ROI '{region.id}' coordinates "
                "must be between 0.0 and 1.0"
            )

        if region.xmin >= region.xmax:
            raise ValueError(
                f"Custom ROI '{region.id}': "
                "xmin must be smaller than xmax"
            )

        if region.ymin >= region.ymax:
            raise ValueError(
                f"Custom ROI '{region.id}': "
                "ymin must be smaller than ymax"
            )