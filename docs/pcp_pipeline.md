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
        +downsample(pcd: open3d.geometry.PointCloud, voxel_size: float) open3d.geometry.PointCloud
    }

    class NormalEstimator {
        +estimate_normals(pcd: open3d.geometry.PointCloud, radius: float, max_nn: int) open3d.geometry.PointCloud
    }
    
    class ClusterExtractor {
        +euclidean_cluster(pcd: open3d.geometry.PointCloud, eps: float, min_points: int, min_size: int) Tuple[open3d.geometry.PointCloud, List[open3d.geometry.PointCloud], int]
    }

    class PointCloudPipeline {
        -processor: PointCloudProcessor
        -normal_estimator: NormalEstimator
        -cluster_extractor: ClusterExtractor
        +__init__(voxel_size: float, cluster_eps: float, cluster_min_points: int, cluster_min_size: int)
        +run_pipeline(pcd: open3d.geometry.PointCloud)
    }

    PointCloudProcessor <|-- PointCloudPipeline : aggregates
    NormalEstimator <|-- PointCloudPipeline : aggregates
    ClusterExtractor <|-- PointCloudPipeline : aggregates
```

# UML Sequence Diagram for Application Entry Point

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
        +loaded_raw_pcd() open3d.geometry.PointCloud // Scope: class
        +empty_pcd() open3d.geometry.PointCloud // Scope: function
    }

    class TestPointCloudProcessing {
        <<Pytest Test Class>>
        +VOXEL_SIZE: float
        +CLUSTER_EPS: float
        +CLUSTER_MIN_POINTS: int
        +CLUSTER_MIN_SIZE: int
        +GOLDEN_LARGEST_CLUSTER_SIZE: float
        +TOLERANCE_PERCENTAGE: float
        
        +test_down_sample_empty_input(empty_pcd: open3d.geometry.PointCloud)
        +test_clustering_empty_input(empty_pcd: open3d.geometry.PointCloud)
        +test_normal_estimator_empty_input(empty_pcd: open3d.geometry.PointCloud)
        +test_voxel_down_sampling_reduces_count(loaded_raw_pcd: open3d.geometry.PointCloud)
        +test_normal_creation_and_dimensionality(loaded_raw_pcd: open3d.geometry.PointCloud)
        +test_clustering_produces_multiple_segments(loaded_raw_pcd: open3d.geometry.PointCloud)
        +test_clustering_regression(loaded_raw_pcd: open3d.geometry.PointCloud)
    }

    class PointCloudLoader {
        // Application Component
        +load_eagle_cloud() open3d.geometry.PointCloud
    }
    
    TestPointCloudProcessing --> Fixture : uses(loaded_raw_pcd)
    TestPointCloudProcessing --> Fixture : uses(empty_pcd)
    Fixture --> PointCloudLoader : calls
    
    note for Fixture "Fixture handles data setup once per a specified scope."
```
