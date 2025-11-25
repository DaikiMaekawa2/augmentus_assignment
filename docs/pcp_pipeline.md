# UML Class Diagram for Point Cloud Pipeline

* `PointCloudLoader`: Responsible solely for retrieving the dataset.
* `PointCloudProcessor`: Handles generic preprocessing (like downsampling).
* `NormalEstimator`: Calculates surface geometry properties.
* `ClusterExtractor`: Executes the segmentation algorithm.
* `PointCloudPipeline`: The primary orchestrator of the application. It manages the sequence of operations (Load -> Process -> Normals -> Cluster -> Render) and holds the configuration parameters.

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

# UML Class Diagram for Pytest Architecture

* `Fixture (loaded_pcd)`: Acts as the setup/teardown utility for the entire test class.
* `TestPointCloudProcessing`: The main container for all unit and regression test methods.

```mermaid
classDiagram
    direction LR

    class Fixture {
        <<pytest fixture>>
        +loaded_pcd()
        // Scope: class
    }

    class TestPointCloudProcessing {
        <<Pytest Test Class>>
        +GOLDEN_LARGEST_CLUSTER_SIZE: int
        +TOLERANCE_PERCENTAGE: float
        
        +test_voxel_down_sampling_reduces_count(loaded_pcd)
        +test_clustering_produces_multiple_segments(loaded_pcd)
        +test_clustering_regression(loaded_pcd)
    }

    class PointCloudLoader {
        // Application Component
        +load_eagle_cloud()
    }
    
    TestPointCloudProcessing --> Fixture : uses(loaded_pcd)
    Fixture --> PointCloudLoader : calls
    
    note for Fixture "Fixture handles expensive setup (data loading) once per test class."
```
