"""
Point Cloud Processing Pipeline Components.

This module contains the core classes required for loading, preprocessing,
analyzing, and visualizing 3D point cloud data using the Open3D library.
"""

from typing import List, Tuple

import numpy as np
import open3d as o3d


class PointCloudLoader:
    """
    Handles the loading of point cloud data from the Open3D dataset utilities.
    """

    def load_eagle_cloud(self) -> o3d.geometry.PointCloud:
        """
        Loads the specific Eagle point cloud dataset provided by Open3D.

        Returns
        -------
        open3d.geometry.PointCloud
            The loaded point cloud object in Open3D's legacy format.
        """

        print("Loading Eagle Point Cloud...")
        data = o3d.data.EaglePointCloud()
        pcd = o3d.t.io.read_point_cloud(data.path)
        pcd_open3d = (
            pcd.to_legacy()
        )  # Convert to legacy Open3D structure for easier processing
        print(f"Loaded {len(pcd_open3d.points)} points.")
        return pcd_open3d


class PointCloudProcessor:
    """
    Performs preprocessing steps on the point cloud data.
    """

    def downsample(
        self, pcd: o3d.geometry.PointCloud, voxel_size: float = 0.01
    ) -> o3d.geometry.PointCloud:
        """
        Reduces the number of points using Voxel Grid Down-sampling.

        Parameters
        ----------
        pcd : open3d.geometry.PointCloud
            The input point cloud to be downsampled.
        voxel_size : float, optional
            The size of the voxel grid (default is 0.01).

        Returns
        -------
        open3d.geometry.PointCloud
            The downsampled point cloud.
        """

        """Reduces the number of points using Voxel Grid Down-sampling."""
        print(f"Applying Voxel Down-sampling with size: {voxel_size}...")
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"Reduced to {len(downsampled_pcd.points)} points.")
        return downsampled_pcd


class NormalEstimator:
    """
    Calculates and assigns surface normals for the point cloud.
    """

    def estimate_normals(
        self, pcd: o3d.geometry.PointCloud, radius: float = 0.05, max_nn: int = 30
    ) -> o3d.geometry.PointCloud:
        """
        Estimates and orients surface normals using a KDTree search.

        Parameters
        ----------
        pcd : open3d.geometry.PointCloud
            The input point cloud (must be downsampled for best results).
        radius : float, optional
            The search radius for finding neighbors (default is 0.05).
        max_nn : int, optional
            The maximum number of neighbors to consider (default is 30).

        Returns
        -------
        open3d.geometry.PointCloud
            The point cloud with normals assigned and oriented.
        """

        """Estimates and orients surface normals."""
        print("Estimating surface normals...")
        # Search parameters for KD-tree based normal estimation
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius, max_nn=max_nn
            )
        )

        return pcd


class ClusterExtractor:
    """
    Segments the point cloud into individual geometric components.
    """

    def euclidean_cluster(
        self,
        pcd: o3d.geometry.PointCloud,
        eps: float = 0.05,
        min_points: int = 10,
        min_size: int = 500,
    ) -> Tuple[o3d.geometry.PointCloud, List[o3d.geometry.PointCloud], int]:
        """
        Performs Euclidean clustering (DBSCAN) on the point cloud.

        The resulting clusters are colored randomly, and noise points are colored black.

        Parameters
        ----------
        pcd : open3d.geometry.PointCloud
            The input point cloud.
        eps : float, optional
            The distance parameter for DBSCAN (maximum distance between two points
            for one to be considered as in the neighborhood of the other) (default is 0.05).
        min_points : int, optional
            The number of points required to form a dense region (default is 10).
        min_size : int, optional
            The minimum size a cluster must be to be retained and colored
            (used to filter noise/debris clusters) (default is 500).

        Returns
        -------
        tuple
            (
            :obj:`open3d.geometry.PointCloud`: The clustered point cloud with colors assigned,
            :obj:`list`: List of individual clustered point cloud objects,
            :obj:`int`: The total count of valid clusters found (excluding noise)
            )
        """

        """Performs Euclidean clustering on the point cloud."""
        print(f"Performing clustering (eps={eps}, min_points={min_points})...")

        if not pcd.points:
            # Return immediately with 0 clusters and empty lists
            return pcd, [], 0

        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
            labels = np.array(
                pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True)
            )

        max_label = labels.max()
        print(f"Point cloud yielded {max_label + 1} total raw clusters.")

        point_colors = np.zeros((len(pcd.points), 3))

        # Generate random colors for cluster labels (index 0 up to max_label)
        # We need max_label + 1 slots for the clusters themselves
        cluster_colors = np.random.rand(max_label + 1, 3)

        valid_cluster_count = 0
        cluster_pcds = []

        for i in range(max_label + 1):
            indices = np.where(labels == i)[0]
            cluster_size = len(indices)

            if cluster_size >= min_size:
                # 1. This is a valid, large cluster. Assign its color to the point_colors array.
                color = cluster_colors[i]
                point_colors[indices] = color

                # 2. Create the individual cluster PCD for the return list
                cluster = pcd.select_by_index(indices)
                cluster.paint_uniform_color(color)
                cluster_pcds.append(cluster)

                valid_cluster_count += 1
            else:
                # 3. This cluster is too small (debris/noise). Points remain colored black (default).
                pass

        pcd.colors = o3d.utility.Vector3dVector(point_colors)
        print(
            f"Post-filtering resulted in {valid_cluster_count} valid clusters (>= {min_size} points)."
        )

        return pcd, cluster_pcds, valid_cluster_count


class PointCloudPipeline:
    """
    Orchestrates the entire point cloud processing workflow
    """

    def __init__(
        self,
        voxel_size: float = 0.01,
        cluster_eps: float = 0.02,
        cluster_min_points: int = 10,
        cluster_min_size: int = 500,
    ):
        """
        Initializes the pipeline with specific parameters and core components.

        Parameters
        ----------
        voxel_size : float, optional
            Voxel size for downsampling (default is 0.01).
        cluster_eps : float, optional
            Epsilon parameter for DBSCAN clustering (default is 0.02).
        cluster_min_points : int, optional
            Minimum points parameter for DBSCAN clustering (default is 10).
        cluster_min_size : int, optional
            Minimum size a cluster must be to be retrained and colored (default is 500).
        """

        self.processor = PointCloudProcessor()
        self.normal_estimator = NormalEstimator()
        self.cluster_extractor = ClusterExtractor()

        self.voxel_size = voxel_size
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points
        self.cluster_min_size = cluster_min_size

    def run_pipeline(self, pcd: o3d.geometry.PointCloud) -> None:
        """
        Executes the full point cloud processing sequentially.

        The method orchestrates the core stages of the pipeline:
        Downsampling, Surface Normal Estimation, Euclidean Clustering

        Parameters
        ----------
        pcd : open3d.geometry.PointCloud
            The point cloud object to be processed
        """

        # 1. Down-sampling

        pcd_downsampled = self.processor.downsample(pcd, self.voxel_size)
        o3d.visualization.draw_geometries([pcd_downsampled], window_name="Downsampled")

        # 2. Normal Estimation

        # Ensures the search sphere is large enough to consistently capture the immediate neighborhood
        search_radius = self.voxel_size * 3.0
        pcd_normals = self.normal_estimator.estimate_normals(
            pcd_downsampled, radius=search_radius
        )
        o3d.visualization.draw_geometries(
            [pcd_normals], point_show_normal=True, window_name="Normals"
        )

        # 3. Clustering
        pcd_clustered, clusters, count = self.cluster_extractor.euclidean_cluster(
            pcd_normals,
            self.cluster_eps,
            self.cluster_min_points,
            self.cluster_min_size,
        )
        o3d.visualization.draw_geometries([pcd_clustered], window_name="Clustered")
