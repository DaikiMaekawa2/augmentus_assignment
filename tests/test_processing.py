"""
Unit and Regression Tests for the Point Cloud Processing Pipeline.

This module validates the core functionality of the point cloud processing steps:
downsampling, clustering segmentation, and output stability. This file uses the
pure Pytest style with fixtures for class-level setup.
"""

import pytest

from pcp.pipeline import (
    ClusterExtractor,
    NormalEstimator,
    PointCloudLoader,
    PointCloudProcessor,
)


# ----------------------------------------------------------------------
# PYTEST FIXTURES
# ----------------------------------------------------------------------


@pytest.fixture(scope="class")
def loaded_pcd():
    """
    Class-scoped fixture to load the Eagle Point Cloud once per test class.

    This ensures the expensive data loading operation is only performed once
    and the loaded data (pcd) is available to all test methods.
    """
    print("\n[Pytest Fixture] Loading Eagle Point Cloud...")
    pcd = PointCloudLoader().load_eagle_cloud()

    # The 'yield' keyword passes the loaded point cloud to the tests.
    yield pcd

    # Code execution resumes here after all tests in the class are finished.
    print("\n[Pytest Fixture] Clean up complete.")


# ----------------------------------------------------------------------
# PYTEST TEST CLASS
# ----------------------------------------------------------------------


class TestPointCloudProcessing:
    """
    Tests the core functionality of the point cloud pipeline components.

    Test methods receive the loaded point cloud data via the 'loaded_pcd' fixture.
    """

    # Define the shared constants for regression testing
    GOLDEN_LARGEST_CLUSTER_SIZE = 251450
    TOLERANCE_PERCENTAGE = 0.01

    def test_voxel_down_sampling_reduces_count(self, loaded_pcd):
        """
        Validates the Voxel Down-sampling process.

        Asserts that the point count is strictly reduced while remaining non-zero.
        """
        processor = PointCloudProcessor()
        initial_count = len(loaded_pcd.points)

        pcd_downsampled = processor.downsample(loaded_pcd, voxel_size=0.01)
        downsampled_count = len(pcd_downsampled.points)

        # Pure Pytest assertion style (simple 'assert' statements)
        assert (
            downsampled_count < initial_count
        ), "Downsampling did not reduce the point count."
        assert downsampled_count > 0, "Downsampling resulted in an empty point cloud."

    def test_clustering_produces_multiple_segments(self, loaded_pcd):
        """
        Verifies that clustering successfully segments the scene into multiple components (> 1 cluster).
        """
        processor = PointCloudProcessor()
        pcd_downsampled = processor.downsample(loaded_pcd, voxel_size=0.01)
        extractor = ClusterExtractor()

        # Normals estimation step is included for robust clustering execution
        normal_estimator = NormalEstimator()
        pcd_normals = normal_estimator.estimate_normals(pcd_downsampled)

        eps = 0.040
        min_points = 35
        min_size = 500

        pcd_clustered, clusters, count = extractor.euclidean_cluster(
            pcd_normals, eps=eps, min_points=min_points, min_size=min_size
        )

        assert count > 1, f"Clustering only produced {count} or fewer segments."

    def test_clustering_regression(self, loaded_pcd):
        """
        Regression test: Asserts that the size of the largest cluster remains stable
        (within the defined tolerance) against a known Golden Standard.
        """

        processor = PointCloudProcessor()
        normal_estimator = NormalEstimator()
        extractor = ClusterExtractor()

        pcd_downsampled = processor.downsample(loaded_pcd, voxel_size=0.01)
        pcd_normals = normal_estimator.estimate_normals(pcd_downsampled)

        eps = 0.040
        min_points = 35
        min_size = 500

        pcd_clustered, clusters_list, count = extractor.euclidean_cluster(
            pcd_normals, eps=eps, min_points=min_points, min_size=min_size
        )

        # Find the size of the largest valid cluster:
        current_largest_size = 0
        if clusters_list:
            # Check the size of the largest cluster in the list of valid clusters
            current_largest_size = max(len(c.points) for c in clusters_list)

        golden_size = TestPointCloudProcessing.GOLDEN_LARGEST_CLUSTER_SIZE
        tolerance = TestPointCloudProcessing.TOLERANCE_PERCENTAGE

        # Calculate max_deviation (the delta/abs tolerance)
        max_deviation = golden_size * tolerance

        # Pytest assertion
        assert current_largest_size == pytest.approx(golden_size, abs=max_deviation), (
            f"Regression failed: Largest cluster size deviated by more than {tolerance * 100}%. "
            f"Expected approx. {golden_size}, got {current_largest_size}."
        )
