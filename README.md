# Multi-Body Static Equilibrium Solver

## Overview

This repository contains the `multi-body-static-equilibrium-solver`, a Python project developed as part of a coursework in Mechanical Transmissions Technology during an MEng degree in Mechanical Engineering at Imperial College London. The project is aimed at solving static equilibrium problems for multi-body systems, particularly focusing on shaft and gear designs. It's a work in progress, reflecting ongoing development and improvements.

### Status
- **Development Stage:** Work in Progress
- **Functionality:** Core calculations functional, plotting features under development
- **License:** MIT License

## Repository Contents

1. **MultiBodyStaticEquilibriumSolver.py**: The main module that includes classes and functions to model solid bodies, coordinate spaces, and solve for static equilibrium in multi-body systems. It features 3D plotting capabilities (currently under development) and example cases to demonstrate the solver's usage.

2. **TransmissionLayoutForces.py**: A specialized script that extends the capabilities of the main solver to model and analyze forces in transmission layouts. It includes functions to simulate gear forces and positions, along with a comprehensive example covering different scenarios in transmission systems.

3. **StaticEquilibriumSolver.py**: An earlier version of the static equilibrium solver. It serves as a simplified model for static equilibrium problems. _Note: This script is currently not recommended for use and is included for reference purposes only._

## Features

- Modeling of solid bodies and coordinate spaces for multi-body systems
- Calculation of static equilibrium conditions in complex mechanical systems
- Customizable inputs for loads, unknowns, and body configurations
- (In-progress) Visualization of the system and forces using 3D plotting

## Installation and Usage

To use this solver, clone the repository and ensure you have Python installed with the required libraries: numpy and matplotlib.

```bash
git clone https://github.com/yourusername/multi-body-static-equilibrium-solver.git
cd multi-body-static-equilibrium-solver
```


Import the main module in your Python script to access its functionalities. Use the example cases provided in `MultiBodyStaticEquilibriumSolver.py` and `TransmissionLayoutForces.py` as templates to model your specific problems.

## Contributing

Contributions are welcome, especially in enhancing the plotting capabilities and expanding the solver's functionalities. Please follow standard pull request procedures.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Disclaimer

This solver is part of an academic project and is still under development. The plotting functionality may not work as expected, and users should primarily rely on the solver for calculations at this stage.
