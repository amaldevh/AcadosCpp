# AcadosCpp

This project provides C++ and pybind11 wrappers that are aware of solver stages.
[acados](https://github.com/acados/acados) generated OCP and simulation solver.
The wrapper is designed for nonlinear model predictive control (NMPC) applications.
In NMPC, the measured state, horizon references, parameters, and warm start are handled as separate pieces of solver data.
Each of these is treated as its own type of solver data.

## Motivation

acados generates fast, optimized C code for a specific model and optimal control problem.
The generated C API offers great performance, but using it directly can create challenges.
Direct use can tie your application to model-specific symbols, dimensions, and other details.
It also requires handling solver lifecycle calls and data updates. Integrating this into a larger
C++ robotics or control stack often means writing extra code to connect everything, which may
need to be updated whenever the model or OCP setup changes.

AcadosCpp provides a stable, unified interface for both C++ and Python around the generated solver.
With this, applications use `ModelOcp` and `ModelSim` instead of working directly with generated C code and acados internals.
This approach avoids the need to manage C capsules or internal acados details.
The typical workflow is:

1. define or change the model and OCP in Python;
2. regenerate the acados solver and AcadosCpp wrapper;
3. recompile and relink the application. The control loop structure and wrapper API stay the same, no matter which model or OCP configuration you use.
However, your application data must still match the new dimensions and cost layout from the generated code.
The wrapper checks these assumptions at the API boundary.
This makes the generated controller much more like a plug-and-play component for your application.
At the same time, it does not hide the stage-wise structure needed by NMPC.
This keeps the flexibility needed for NMPC.

## What the wrapper supports

- The measured state constrains only OCP stage 0.
- Running references can differ at every stage, and the terminal reference has
  its own dimension.
- Model parameters can be updated per stage; global parameters are supported.
- State and control initial guesses can be set per node or as trajectories.
- A successful solution is cached and can be shifted one stage for the next
  NMPC iteration.
- The complete optimal state/control trajectories and solver statistics are
  available in C++ and Python.
- SQP-RTI preparation and feedback phases can be selected explicitly.
- The generated wrapper is a CMake subproject with stable `AcadosCpp::ocp` and
  `AcadosCpp::sim` targets. It retains acados' model-specific CMake source list,
  so source selection is not hardcoded to ERK dynamics or nonlinear
  least-squares costs.

## Requirements

- acados, with `ACADOS_ROOT` pointing to its installation
- Python 3.9+
- CMake 3.18+
- a C++17 compiler
- `jinja2` and `pybind11`; the example also uses `acados_template`, CasADi,
  NumPy, SciPy, and Matplotlib

## Generate and build

First generate an OCP solver and simulation solver with the CMake builder from
`acados_template`. 

Then generate and build the wrappers:

```bash
python3 generate_cpp_ocp.py \
  --model_name quadrotor \
  --c_generated_code_dir /tmp/c_generated_code_ocp \
  --output_dir ./examples/cpp_quadrotor_ocp
```

You can use `--no-build` to generate and copy the wrapper files without compiling them.
This is helpful if you use another build system or a cross-compiler for the final build.
The normal build configures CMake with Python bindings enabled and installs the
resulting libraries and modules in the output directory.

## Use from a CMake application

The generated directory can be added directly to another project. The only
solver-specific value the downstream build needs is its location:

```cmake
set(ACADOS_CPP_GENERATED_DIR "/path/to/cpp_my_model_ocp")
add_subdirectory(
    "${ACADOS_CPP_GENERATED_DIR}"
    "${CMAKE_BINARY_DIR}/acados_cpp_generated")

add_executable(my_controller controller.cc)
target_link_libraries(my_controller PRIVATE AcadosCpp::ocp AcadosCpp::sim)
```

The target names, wrapper header names (`model_ocp.hh` and `model_sim.hh`),
include paths, and link dependencies remain unchanged when the generated model
changes. Set `ACADOS_CPP_BUILD_PYTHON=ON` before `add_subdirectory()` only when
the downstream build also needs the pybind11 modules.

## Standard NMPC loop

For the common least-squares layout `y = [x, u]`, with terminal output
`y_e = x`, supply one state reference per node and one control reference per
shooting interval:

```cpp
ModelOcp ocp;
ModelSim plant;

std::vector<double> x = /* measured initial state */;
std::vector<double> u_equilibrium = /* feed-forward input */;
std::vector<std::vector<double>> xrefs(ocp.horizon() + 1, target);
std::vector<std::vector<double>> urefs(ocp.horizon(), u_equilibrium);

ocp.initialize_guess(x, u_equilibrium);
plant.set_x0(x);

while (running) {
    // The convenience overload shifts the preceding solution, fixes stage 0
    // to the new measurement, updates all references, and solves.
    const auto& u = ocp.solve(x, xrefs, urefs);
    x = plant.step(u);  // replace with the next real state measurement
}
```

If your path changes, update or shift `xrefs` and `urefs` before each call. Do not
set every state node to the measured `x`: that discards the predicted trajectory
and leads to a poor warm start after the first solve.

The equivalent Python API accepts lists or other pybind11-convertible
sequences:

```python
ocp.initialize_guess(x, u_equilibrium)
while running:
    u = ocp.solve(x, xrefs, urefs)
    x = plant.step(u)
```

## General cost outputs

An acados cost output does not have to equal `[x, u]`. For arbitrary LINEAR_LS,
NONLINEAR_LS, or EXTERNAL cost definitions, use raw stage references:

```cpp
ocp.set_yref(0, yref_0);             // length MODEL_NY0
ocp.set_yref(stage, yref);           // length MODEL_NY, stage 1..N-1
ocp.set_yref(ocp.horizon(), yref_e); // length MODEL_NYN
```

`set_yref_trajectory()` accepts all `N+1` vectors and validates each stage's
dimension. The state/control convenience methods reject incompatible cost
dimensions rather than silently assuming a layout.

## Parameters and guesses

```cpp
ocp.set_parameters(stage, p);
ocp.set_parameter_trajectory(p_horizon); // N+1 parameter vectors
ocp.set_global_parameters(p_global);

ocp.set_state_guess(stage, x_guess);
ocp.set_control_guess(stage, u_guess);
ocp.set_state_guess(x_guess_horizon);    // N+1 state vectors
ocp.set_control_guess(u_guess_horizon);  // N control vectors
ocp.shift_warm_start();
```

The older `set_xinit`, `set_uinit`, and two-vector `set_yref` methods are still available for compatibility.
They initialize or apply values to every relevant node and should
usually only be used when starting the controller.

## SQP-RTI

With an OCP generated using `nlp_solver_type = "SQP_RTI"`, phase 1 can be run
before the newest measurement arrives and phase 2 after stage 0 is updated:

```cpp
ocp.shift_warm_start();
ocp.set_reference_trajectory(xrefs, urefs);
ocp.set_rti_phase(1);  // preparation
ocp.solve();

ocp.set_x0(measured_x);
ocp.set_rti_phase(2);  // feedback
const auto& u = ocp.solve();
```

Use phase 0 for a complete RTI iteration in one `solve()` call. Do not select
RTI phases for a solver generated with a different NLP method.

## Main API

| Method/property | Purpose |
|---|---|
| `set_x0(x)` | Fix stage-0 constrained state components |
| `set_yref(stage, y)` | Set a dimension-checked raw stage reference |
| `set_reference_trajectory(xs, us)` | Set an `[x,u]` tracking horizon |
| `set_parameter_trajectory(ps)` | Set stage-varying model parameters |
| `initialize_guess(x, u)` | Initialize all nodes at controller startup |
| `shift_warm_start()` | Shift the cached optimal trajectory |
| `solve()` | Solve the currently configured OCP |
| `state_trajectory`, `control_trajectory` | Cached optimal solution |
| `status`, `solve_time`, `sqp_iterations`, `kkt_norm_inf` | Solver diagnostics |

For full Python and C++ instructions, see the [examples guide](examples/README.md).
It covers all closed-loop usage details.
