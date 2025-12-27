# AcadosCpp

An object-oriented C++ wrapper around [acados](https://github.com/acados/acados) OCP (Optimal Control Problem) and Sim (Simulation) solvers, with automatic code generation and Python bindings.


## Overview

AcadosCpp streamlines the process of using acados solvers in C++ applications by providing:

- **Automatic C++ wrapper generation** from acados Python-generated C code
- **Object-oriented interface** (`ModelOcp` and `ModelSim` classes) for clean integration
- **Python bindings** via pybind11 for rapid prototyping and testing
- **Jinja2 templates** for customizable code generation

This is particularly useful for Model Predictive Control (MPC) applications in robotics, autonomous systems, and other real-time control scenarios.

## Features

- 🚀 **High Performance**: Leverages acados' efficient solvers (HPIPM, BLASFEO) for real-time MPC
- 🔧 **Easy Integration**: Simple `ModelOcp::solve()` and `ModelSim::step()` interfaces
- 🐍 **Python Interoperability**: Generated Python modules for testing and validation
- 📦 **Automated Build System**: Single command generates and compiles all necessary libraries
- 🎯 **Flexible Templates**: Jinja2-based templates allow customization of generated code

## Requirements

### Dependencies

- **acados** (with environment variable `ACADOS_ROOT` set)
- **Python 3.8+**
- **C++17** compatible compiler (GCC/Clang)
- **Python packages**:
  ```
  acados_template
  casadi
  numpy
  scipy
  jinja2
  pybind11
  ```

### Installation

1. **Install acados** following the [official installation guide](https://docs.acados.org/installation/index.html)

2. **Set environment variable**:
   ```bash
   export ACADOS_ROOT=/path/to/acados
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_ROOT/lib
   ```

3. **Install Python dependencies**:
   ```bash
   pip install acados_template casadi numpy scipy jinja2 pybind11
   ```

4. **Clone this repository**:
   ```bash
   git clone https://github.com/amaldevh/AcadosCpp.git
   cd AcadosCpp
   ```

## Usage

### Quick Start

The workflow consists of two main steps:

1. **Define your OCP in Python** using acados_template
2. **Generate C++ wrappers** using `generate_cpp_ocp.py`

### Example: Quadrotor MPC

See the complete example in the `examples/` directory.

#### 1. Define the dynamics and OCP (Python)

```python
import acados_template as at
import casadi as ca
import numpy as np

# Define your model dynamics
ocp = at.AcadosOcp()
model = ocp.model
model.name = 'quadrotor'
model.x = ca.SX.sym('x', 13)  # states
model.u = ca.SX.sym('u', 4)   # controls
model.f_expl_expr = your_dynamics(model.x, model.u)

# Configure OCP settings...
ocp_solver = at.AcadosOcpSolver(ocp)
```

#### 2. Generate C++ code

```python
from generate_cpp_ocp import generate_code

generate_code(
    model_name='quadrotor',
    c_generated_code_dir='/path/to/acados/generated/code',
    output_dir='./cpp_quadrotor_ocp'
)
```

Or via command line:
```bash
python generate_cpp_ocp.py \
    --model_name quadrotor \
    --c_generated_code_dir /tmp/c_generated_code_ocp \
    --output_dir ./cpp_quadrotor_ocp
```

#### 3. Use in C++

```cpp
#include "model_ocp.hh"
#include "model_sim.hh"

int main() {
    ModelOcp ocp;
    ModelSim sim;
    
    std::vector<double> x0 = {0.0, 0.0, 0.0, ...};  // Initial state
    std::vector<double> xdes = {5.0, 5.0, 5.0, ...}; // Desired state
    
    sim.set_x0(x0);
    sim.set_dt(0.001);  // 1ms timestep
    
    // Control loop
    for (int i = 0; i < num_iterations; ++i) {
        const auto& u_opt = ocp.solve(x0, xdes);
        x0 = sim.step(u_opt);
    }
    
    return 0;
}
```

#### 4. Compile your application

```bash
cd examples
make control_loop
./control_loop
```

### Using Python Bindings

The generated Python modules can be used for testing:

```python
import sys
sys.path.append('./cpp_quadrotor_ocp')

from quadrotor_ocp_py import ModelOcp
from quadrotor_sim_py import ModelSim

ocp = ModelOcp()
sim = ModelSim()

x0 = [0.0, 0.0, 0.0, ...]
xdes = [5.0, 5.0, 5.0, ...]

sim.set_x0(x0)
sim.set_dt(0.001)

u_opt = ocp.solve(x0, xdes)
x_next = sim.step(u_opt)
```

## Project Structure

```
AcadosCpp/
├── generate_cpp_ocp.py      # Main code generation script
├── templates/               # Jinja2 templates for C++ code
│   ├── model_ocp.cc.j2      # OCP solver implementation
│   ├── model_ocp.hh.j2      # OCP solver header
│   ├── model_ocp_py.cc.j2   # Python bindings for OCP
│   ├── model_sim.cc.j2      # Simulator implementation
│   ├── model_sim.hh.j2      # Simulator header
│   └── model_sim_py.cc.j2   # Python bindings for Sim
└── examples/
    ├── quadrotor_model.py   # Example: Quadrotor dynamics & OCP setup
    ├── control_loop.py      # Example: Python closed-loop simulation
    ├── cpp_control_loop.cc  # Example: C++ closed-loop simulation
    └── Makefile             # Build configuration for examples
```

## API Reference

### ModelOcp Class

| Method | Description |
|--------|-------------|
| `ModelOcp()` | Constructor, initializes the acados OCP solver |
| `set_x0(x0)` | Set initial state constraint |
| `set_xinit(xinit)` | Set initial guess for state trajectory |
| `set_uinit(uinit)` | Set initial guess for control trajectory |
| `set_yref(xref, uref)` | Set reference trajectory for cost function |
| `solve()` | Solve OCP and return optimal control |
| `solve(x0, xdes)` | Convenience method: set x0, reference, and solve |

### ModelSim Class

| Method | Description |
|--------|-------------|
| `ModelSim()` | Constructor, initializes the acados integrator |
| `set_x0(x0)` | Set current state |
| `set_dt(dt)` | Set integration timestep |
| `step(u)` | Integrate dynamics with control input, return next state |

## Generated Libraries

After running `generate_cpp_ocp.py`, the following libraries are created:

| Library | Description |
|---------|-------------|
| `lib<model>_ocp_fncs.so` | Acados-generated C functions |
| `lib<model>_ocp.so` | C++ OCP wrapper |
| `lib<model>_sim.so` | C++ Simulation wrapper |
| `<model>_ocp_py.*.so` | Python OCP module |
| `<model>_sim_py.*.so` | Python Sim module |

## Performance

The generated code is optimized for real-time performance. Example benchmark on a quadrotor MPC problem (13 states, 4 controls, N=10 horizon):

- **C++ closed-loop**: ~5000+ Hz
- **Python bindings**: ~4000+ Hz

## Troubleshooting

### Common Issues

1. **`ACADOS_ROOT` not set**
   ```bash
   export ACADOS_ROOT=/path/to/acados
   ```

2. **Library not found at runtime**
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/generated/libs:$ACADOS_ROOT/lib
   ```

3. **Python module import fails**
   - Ensure the generated library directory is in `sys.path`
   - Check that all `.so` files were generated successfully

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


## Acknowledgments

- [acados](https://github.com/acados/acados) - The underlying optimal control solver
- [CasADi](https://web.casadi.org/) - Symbolic framework for automatic differentiation
- [pybind11](https://github.com/pybind/pybind11) - C++/Python bindings

## Citation

If you use this project in your research, please consider citing:

```bibtex
@software{acadoscpp,
  title = {AcadosCpp: Object-Oriented C++ Wrapper for acados},
  url = {https://github.com/amaldevh/AcadosCpp},
  year = {2024}
}
```
