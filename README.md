# augmentus_assignment

[UML Diagram](docs/pcp_pipeline.md)

## Installation and Setup

This project requires Python 3.8+ and uses a virtual environment for dependency management.

### Prerequisites

1. Create Virtual Environment (venv):

```bash
$ python3 -m venv venv_open3d
$ source venv_open3d/bin/activate
```

2. Install Dependencies:

```bash
(venv_open3d) pip install open3d numpy pytest pytest-flake8 black isort sphinx sphinx-rtd-theme sphinx-mermaid
```

## Usage

The core workflow is executed via the run_eagle_pipeline.py script.

### Run the main pipeline script from the project root

```bash
(venv_open3d) python pcp/run_eagle_pipeline.py
```

## Testing and Code Quality

The project utilizes Pytest for a comprehensive testing suite that includes functionality checks, regression testing, and linting.

### Code Formatting

The codebase adheres to strict style standards enforced by Black and isort.

To format the entire project, run:

```bash
(venv_open3d) black .
(venv_open3d) isort .
```

### Running Tests

Run the following command from the project root to execute the entire test suite:

```bash
(venv_open3d) pytest
```

The test suite performs three actions:

1. Linting (Flake8): Checks all Python source files for PEP 8 style violations.

2. Unit Tests: Verifies core functionality, such as ensuring Voxel Down-sampling correctly reduces the point count and clustering produces multiple segments.

3. Regression Test: Compares the size of the largest cluster against a known Golden Standard value to ensure stability against code changes.
