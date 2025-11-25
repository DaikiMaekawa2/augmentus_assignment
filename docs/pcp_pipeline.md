# UML Class Diagram for Point Cloud Pipeline

* PointCloudLoader: Responsible solely for retrieving the dataset.
* PointCloudProcessor: Handles generic preprocessing (like downsampling).
* NormalEstimator: Calculates surface geometry properties.
* ClusterExtractor: Executes the segmentation algorithm.
* PointCloudPipeline: The primary orchestrator of the application. It manages the sequence of operations (Load -> Process -> Normals -> Cluster -> Render) and holds the configuration parameters.

```mermaid
classDiagram
    direction TB
    
    class PointCloudLoader {
        +load_eagle_cloud() open3d.geometry.PointCloud
    }

    class PointCloudProcessor {
        +downsample(pcd, voxel_size) open3d.geometry.PointCloud
    }

    class NormalEstimator {
        +estimate_normals(pcd, radius, max_nn) open3d.geometry.PointCloud
    }
    
    class ClusterExtractor {
        +euclidean_cluster(pcd, eps, min_points) tuple
    }

    class PointCloudPipeline {
        -loader: PointCloudLoader
        -processor: PointCloudProcessor
        -normal_estimator: NormalEstimator
        -cluster_extractor: ClusterExtractor
        +__init__(voxel_size, eps, min_points)
        +save_render(pcd, filename)
        +run_pipeline()
    }

    PointCloudLoader <|-- PointCloudPipeline : aggregates
    PointCloudProcessor <|-- PointCloudPipeline : aggregates
    NormalEstimator <|-- PointCloudPipeline : aggregates
    ClusterExtractor <|-- PointCloudPipeline : aggregates
```
