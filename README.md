# cansat_modeling

This repository contains a Python-based simulator to model the descent trajectory of a CanSat (or similar system) from high altitude, considering different types of wind and varying levels of lateral control (`GR`). The simulation uses a physics-based approach including air resistance and orientation manipulation to observe landing dispersion and descent paths. The equations are obtained from the following research: 

Fields, T. D., & Yakimenko, O. A. (2017, June 7). The use of a steerable single-actuator cruciform parachute for targeted payload return. IEEE Aerospace Conference Proceedings. https://doi.org/10.1109/AERO.2017.7943787

### Code Structure

- `dropping()`: Main simulation function. Executes multiple drop attempts with active manipulation under selected wind conditions.
- `wind_components()`: Generates wind components in X and Y directions.
- `main`: Runs simulations for different values of `GR` and wind types.
- 3D visualization of trajectories and 2D scatter of landing points.

### Wind Types Simulated

- `sinuidal`: periodic wind (sinusoidal pattern).
- `noise`: wind with Gaussian noise.
- `turbulence`: smoothed random wind simulating gusty turbulence.

### How to Run

1. Clone the repository:
   git clone https://github.com/cdtm15/cansat_modeling.git
   cd cansat_modeling

2. Make sure you have Python 3.8+ and install the dependencies:
   pip install numpy matplotlib pandas scipy

3. Run the main script:
   python cansat_fall_parachute.py

> You can modify parameters like `num_intentos`, `GR` and `wind_type` inside the script to explore different scenarios.

### Computed Metrics

For each drop attempt, the simulator calculates:
- Time of descent
- Terminal velocity
- Control efficiency (based on landing dispersion)

These metrics are automatically aggregated in a DataFrame for analysis.

### Visualizations

- 3D trajectories for each combination of control level (`GR`) and wind type.
- 2D landing points, colored by `GR` value.
- 3x3 subplot grid to visually compare all scenarios.

### Files

- `cansat_fall_parachute.py`: main simulation and visualization script.
- `README.md`: project description.

### Credits and Purpose

This project was developed for educational and scientific exploration, especially to visualize how different wind types affect the dispersion of a satellite with limited control capabilities.
