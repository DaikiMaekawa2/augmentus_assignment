"""
Point Cloud Processing Pipeline Components.

This module contains the core classes required for loading, preprocessing,
analyzing, and visualizing 3D point cloud data using the Open3D library.
"""

import open3d as o3d


class PointCloudLoader:
    """
    Handles the loading of point cloud data from the Open3D dataset utilities.
    """

    def load_eagle_cloud(self):
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

    def downsample(self, pcd, voxel_size=0.01):
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


class PointCloudPipeline:
    """
    Orchestrates the entire point cloud processing workflow, from loading to visualization.
    """

    def __init__(self, voxel_size=0.01, cluster_eps=0.02, cluster_min_points=10):
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
        """

        self.loader = PointCloudLoader()
        self.processor = PointCloudProcessor()

        self.voxel_size = voxel_size
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points

    def save_render(self, pcd, filename):
        """
        Renders the point cloud to an image file, ensuring proper camera setup for headless environments.

        Parameters
        ----------
        pcd : open3d.geometry.PointCloud
            The point cloud to be rendered.
        filename : str
            The name of the output image file (e.g., 'output.png').
        """

        """Helper to render and save the point cloud as an image using the Visualizer."""

        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1280, height=960)
        vis.add_geometry(pcd)

        ctr = vis.get_view_control()
        ctr.set_lookat(pcd.get_center())
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        ctr.set_front([0.5, -0.5, -1.0])
        ctr.set_up([0, 0, 1])
        ctr.set_zoom(0.5)
        vis.capture_screen_image(filename)
        vis.destroy_window()

        print(f"Render saved as {filename}")

    def run_pipeline(self):
        """
        Executes the full point cloud processing sequence: Load -> Downsample -> Normals -> Cluster -> Render.
        """

        pcd_original = self.loader.load_eagle_cloud()

        # Interactive View
        o3d.visualization.draw_geometries([pcd_original], window_name="Original")

        # Down-sampling
        pcd_downsampled = self.processor.downsample(pcd_original, self.voxel_size)
        self.save_render(pcd_downsampled, "render_downsampled.png")
        print("render_downsampled.png saved.")

        # Interactive View
        o3d.visualization.draw_geometries([pcd_downsampled], window_name="Downsampled")


if __name__ == "__main__":
    pipeline = PointCloudPipeline()
    pipeline.run_pipeline()
