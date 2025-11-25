# UML Class Diagram for Core Processing Pipeline

* `PointCloudLoader`: Responsible solely for retrieving the dataset.
* `PointCloudProcessor`: Handles generic preprocessing (like downsampling).
* `NormalEstimator`: Calculates surface geometry properties.
* `ClusterExtractor`: Executes the segmentation algorithm.
* `PointCloudPipeline`: The primary orchestrator of the processing flow.

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
        -processor: PointCloudProcessor
        -normal_estimator: NormalEstimator
        -cluster_extractor: ClusterExtractor
        +__init__(voxel_size, eps, min_points)
        +save_render(pcd, filename)
        +run_pipeline(pcd)
    }

    PointCloudProcessor <|-- PointCloudPipeline : aggregates
    NormalEstimator <|-- PointCloudPipeline : aggregates
    ClusterExtractor <|-- PointCloudPipeline : aggregates
```

# Application Entry Point

This layer handles the specific configuration needed to run the application for the Eagle dataset.

```mermaid
sequenceDiagram
    participant E as EntryPoint (run_eagle_pipeline.py)
    participant L as PointCloudLoader
    participant P as PointCloudPipeline

    Note over E: Data Acquisition & Transformation
    E->L: __init__()
    E->E: transformed_pcd = load_and_transform_eagle_data()

    Note over E: Initialize Generic Pipeline
    E->P: __init__(config params)

    E->P: run_pipeline(transformed_pcd)
    Note over P: Executes Downsample -> Normals -> Cluster
    P-->E: Pipeline Complete
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
