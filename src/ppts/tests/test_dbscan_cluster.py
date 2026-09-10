import numpy as np
import pytest

from algorithms.dbscan_cluster import DBSCANCluster


def test_empty_input():
    """Should return no clusters for empty input."""
    clusterer = DBSCANCluster()

    points = np.empty((0, 2))
    clusters = clusterer.cluster(points)

    assert clusters == []


def test_single_cluster():
    """Points close together should form one cluster."""
    clusterer = DBSCANCluster(eps=0.2, min_samples=3)

    points = np.array([
        [0.00, 0.00],
        [0.05, 0.02],
        [0.10, 0.01],
        [0.08, 0.04],
        [0.03, 0.06],
    ])

    clusters = clusterer.cluster(points)

    assert len(clusters) == 1
    assert len(clusters[0].points) == 5


def test_two_clusters():
    """Separated point groups should create two clusters."""
    clusterer = DBSCANCluster(eps=0.2, min_samples=3)

    points = np.array([
        [0.0, 0.0],
        [0.1, 0.0],
        [0.0, 0.1],
        [5.0, 5.0],
        [5.1, 5.0],
        [5.0, 5.1],
    ])

    clusters = clusterer.cluster(points)

    assert len(clusters) == 2

    cluster_sizes = sorted([len(c.points) for c in clusters])
    assert cluster_sizes == [3, 3]


def test_noise_points():
    """Isolated points should be classified as noise."""
    clusterer = DBSCANCluster(eps=0.15, min_samples=3)

    points = np.array([
        [0.0, 0.0],
        [0.05, 0.0],
        [0.0, 0.05],
        [10.0, 10.0],   # Noise
    ])

    clusters = clusterer.cluster(points)

    assert len(clusters) == 1
    assert len(clusters[0].points) == 3


def test_all_noise():
    """All isolated points should produce no clusters."""
    clusterer = DBSCANCluster(eps=0.1, min_samples=3)

    points = np.array([
        [0, 0],
        [2, 2],
        [4, 4],
        [6, 6],
    ])

    clusters = clusterer.cluster(points)

    assert clusters == []


def test_cluster_centroid():
    """Verify centroid calculation."""
    clusterer = DBSCANCluster(eps=0.3, min_samples=3)

    points = np.array([
        [1.0, 1.0],
        [1.2, 1.0],
        [1.0, 1.2],
        [1.1, 1.1],
    ])

    clusters = clusterer.cluster(points)

    assert len(clusters) == 1

    centroid = clusters[0].centroid

    np.testing.assert_allclose(
        centroid,
        np.array([1.075, 1.075]),
        atol=1e-3
    )


@pytest.mark.parametrize(
    "eps,min_samples,expected_clusters",
    [
        (0.2, 2, 2),
        (0.2, 3, 2),
        (0.05, 2, 0),
    ]
)
def test_parameter_variations(eps, min_samples, expected_clusters):
    """Check clustering with different DBSCAN parameters."""
    clusterer = DBSCANCluster(
        eps=eps,
        min_samples=min_samples
    )

    points = np.array([
        [0, 0],
        [0.1, 0],
        [5, 5],
        [5.1, 5],
    ])

    clusters = clusterer.cluster(points)

    assert len(clusters) == expected_clusters