# Quadrotor NMPC example

This directory contains matching Python and C++ closed-loop examples for a
13-state quadrotor controlled by four inputs. The example demonstrates the
stage-aware API: the measured state constrains only stage 0, references cover
the complete prediction horizon, and each solve warm-starts from the shifted
previous solution.

## Files

| File | Purpose |
|---|---|
| `quadrotor_model.py` | Defines the CasADi dynamics and acados OCP |
| `control_loop.py` | Generates, builds, and runs the Python-bound controller |
| `cpp_control_loop.cc` | Runs the same NMPC structure directly from C++ |
| `CMakeLists.txt` | Builds the C++ closed-loop executable against stable AcadosCpp targets |
| `plot_simulation.py` | Plots a CSV log written by the C++ executable |

Generated solver and wrapper files are placed in `cpp_quadrotor_ocp/`.

## Prerequisites

Install acados and the Python dependencies described in the repository's main
[README](../README.md), then configure the runtime environment:

```bash
export ACADOS_ROOT=/absolute/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

All commands below assume the current directory is `examples/`:

```bash
cd examples
```

## Run the Python closed loop

```bash
python3 control_loop.py
```

This command performs the complete workflow:

1. creates the quadrotor OCP with `acados_template`;
2. generates and compiles the C++, acados, and pybind11 libraries;
3. runs five seconds of closed-loop simulation at a 1 ms plant timestep;
4. prints timing statistics and plots state and control histories.

The controller prediction horizon has 10 shooting intervals over 0.2 seconds.
The 1 ms timestep belongs to the separate plant simulator and is intentionally
smaller than an OCP shooting interval.

## Build and run the C++ closed loop

Generate the acados CMake project and wrappers first:

```bash
python3 quadrotor_model.py
python3 ../generate_cpp_ocp.py \
  --model_name quadrotor \
  --c_generated_code_dir /tmp/c_generated_code_ocp \
  --output_dir "$PWD/cpp_quadrotor_ocp"
```

This uses the default `std::vector<double>` interface. Add
`--vector_type eigen` to generate an `Eigen::VectorXd` interface instead.
Eigen3 must then be available to CMake; the generated targets propagate
`Eigen3::Eigen` to this example automatically. The C++ loop uses the generated
`ModelOcp::Vector` and `ModelOcp::VectorArray` aliases, so no source changes are
needed when switching modes. The scalar type remains `double` in both modes to
match acados.

Then configure, compile, and run the controller:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DACADOS_CPP_GENERATED_DIR="$PWD/cpp_quadrotor_ocp"
cmake --build build --parallel
./build/control_loop
```

The executable prints average solve-loop timing and writes
`simulation_log.csv` with the state, applied control, and desired state at each
iteration.

Plot the log interactively, or save it for a headless run:

```bash
python3 plot_simulation.py
python3 plot_simulation.py --save simulation.png --no-show
```

`CMakeLists.txt` changes only `ACADOS_CPP_GENERATED_DIR` when selecting another
generated solver. Application targets always link `AcadosCpp::ocp` and
`AcadosCpp::sim`; model-specific library names, include paths, rpaths, and
acados dependencies are supplied by the generated CMake project.

## What happens during one NMPC iteration

The example initializes the solver only once:

```cpp
ModelOcp::VectorArray xrefs(ocp.horizon() + 1, xdes);
ModelOcp::VectorArray urefs(ocp.horizon(), u_hover);
ocp.initialize_guess(x0, u_hover);
```

Each subsequent call performs the normal receding-horizon operations:

```cpp
const auto& u = ocp.solve(x0, xrefs, urefs);
x0 = sim.step(u);
```

`solve(x0, xrefs, urefs)` shifts the preceding optimal state/control
trajectory, fixes the new measurement at stage 0, applies all running and
terminal references, and solves. Only the first optimized input is applied to
the plant.

For path tracking, replace the rows of `xrefs` and `urefs` before each call.
Their required sizes are:

- `xrefs`: `N + 1` vectors of length `NX`, including the terminal node;
- `urefs`: `N` vectors of length `NU`.

This convenience layout assumes running cost output `y=[x,u]` and terminal
output `y_e=x`. For another cost definition, use `set_yref(stage, value)` or
`set_yref_trajectory()` directly.

## SQP-RTI split execution

The example OCP uses SQP-RTI. Applications that can perform preparation before
the newest measurement arrives may split the iteration:

```cpp
ocp.shift_warm_start();
ocp.set_reference_trajectory(xrefs, urefs);
ocp.set_rti_phase(1);
ocp.solve();

ocp.set_x0(measured_x);
ocp.set_rti_phase(2);
const auto& u = ocp.solve();
```

Use `set_rti_phase(0)` to return to a complete preparation-and-feedback call.

## Troubleshooting

- If an acados library cannot be loaded, verify `ACADOS_ROOT` and
  `LD_LIBRARY_PATH`.
- If CMake cannot find the generated project, run the two generation commands
  above or set `-DACADOS_CPP_GENERATED_DIR=/absolute/path/to/output`.
- If Python cannot import the generated module, run `control_loop.py` from this
  directory or add `cpp_quadrotor_ocp/` to `PYTHONPATH`.
- If the wrapper build cannot find Python or pybind11, install the development
  package for the selected Python interpreter and install `pybind11` into that
  interpreter.
- Regenerate the wrapper after changing dimensions, costs, constraints, or
  solver options in `quadrotor_model.py`.
