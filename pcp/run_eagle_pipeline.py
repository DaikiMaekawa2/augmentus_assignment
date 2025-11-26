"""
Application Entry Point

This script handles the specific configuration for the Eagle Point Cloud
dataset, applies the required initial rotation, and executes
PointCloudPipeline found in the 'pcp' package.
"""

import numpy as np
import open3d as o3d
from pcp.pipeline import PointCloudPipeline, PointCloudLoader


def load_and_transform_eagle_data() -> o3d.geometry.PointCloud:
    """
    Loads the Eagle Point Cloud and applies the specific rotation transformation
    required for visualization consistency.

    Returns
    -------
    open3d.geometry.PointCloud
        The loaded point cloud object after the specific initial
        transformation has been applied.
    """

    loader = PointCloudLoader()
    pcd_original = loader.load_eagle_cloud()

    # Apply specific transformation for visualization alignment.
    R = pcd_original.get_rotation_matrix_from_xyz((-np.pi, -np.pi / 4, 0))
    pcd_transformed = pcd_original.rotate(R, center=pcd_original.get_center())

    print("Initial transformation applied to the Eagle Point Cloud.")
    return pcd_transformed


if __name__ == "__main__":
    transformed_pcd = load_and_transform_eagle_data()

    pipeline = PointCloudPipeline(
        voxel_size=0.01, cluster_eps=0.045, cluster_min_points=35, cluster_min_size=500
    )
    pipeline.run_pipeline(transformed_pcd)

    print("\nrun_eagle_pipeline.py execution complete.")
