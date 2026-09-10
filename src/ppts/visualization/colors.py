#!/usr/bin/env python3

from std_msgs.msg import ColorRGBA


class Colors:
    """
    Standard colors used throughout PPTS visualization.
    """

    RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

    GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    BLUE = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

    YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

    CYAN = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)

    MAGENTA = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)

    WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

    BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)

    GRAY = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)

    ORANGE = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)

    # Palette used for cluster visualization
    CLUSTER_COLORS = [
        RED,
        GREEN,
        BLUE,
        YELLOW,
        CYAN,
        MAGENTA,
        ORANGE,
        WHITE,
    ]

    @staticmethod
    def cluster_color(cluster_id: int) -> ColorRGBA:
        """
        Returns a deterministic color for a cluster.

        Parameters
        ----------
        cluster_id : int

        Returns
        -------
        ColorRGBA
        """

        return Colors.CLUSTER_COLORS[
            cluster_id % len(Colors.CLUSTER_COLORS)
        ]