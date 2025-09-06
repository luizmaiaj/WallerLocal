# Robot Navigation GP

A Python implementation of genetic programming for robot navigation.

## Requirements

- Python 3.8+
- Dependencies listed in requirements.txt
- ImageMagick (for visualization)

## Installation

1. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Install ImageMagick for your operating system (required for visualization)

## Usage

Run the main script:
```bash
python src/main.py
```

## Structure

- `src/`
  - `main.py`: Main script
  - `constants.py`: Configuration constants
  - `environment.py`: Environment class
  - `robot.py`: Robot class
  - `gp/`: Genetic Programming package
    - `engine.py`: GP engine using DEAP
    - `primitives.py`: Robot GP primitives
  - `visualization/`: Visualization utilities
    - `tracking.py`: Path tracking visualization
- `data/`: Evolution statistics output
- `paths/`: Path visualizations output
- `robots/`: Final robot solutions output

## License

See LICENSE file.