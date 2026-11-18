# Unicycle Robot: Modelling and Control

MATLAB project developed for the **Digital Control Systems** course (academic year 2024/2025). The project addresses the regulation of a unicycle mobile robot to the origin and compares a discrete-time sliding-mode controller with two continuous-time control approaches.

## Project content

The project includes three controller implementations:

1. **Discrete-time sliding-mode control** — based on the discrete-time chained-form model, with multirate input sampling, bounded control inputs and actuator saturation.
2. **Polar-coordinate regulation** — a continuous-time unicycle controller designed using the polar error coordinates \(\rho\), \(\gamma\) and \(\delta\), simulated with sample-and-hold inputs.
3. **Dynamic feedback linearization** — a controller based on a dynamically extended unicycle model, also simulated with sample-and-hold inputs.

The accompanying report discusses the unicycle kinematic and chained-form models, discretization, sliding surfaces, input saturation and domain of attraction (DOA), a perturbed-unicycle case, simulation results and a comparison of the three controllers.

## Repository structure

```text
.
├── README.md
├── docs/
│   ├── Digital_Control_Systems.pdf
│   ├── SlideDigital.pdf
│   ├── project_overview.md
│   ├── implementation_notes.md
│   └── references.md
├── src/
   ├── controllers/
   │   ├── sliding_mode.m
   │   ├── polar_coordinates.m
   │    └── dynamic_feedback_linearization.m
   └── models/
       ├── unicycle_model.m
       └── extended_unicycle.m
```

### Project documents

- [Written report](docs/Digital_Control_Systems.pdf) — full project report, including the theoretical development, simulations and conclusions.
- [Presentation](docs/SlideDigital.pdf) — presentation of the project objectives, control designs, simulation outcomes and comparison.
- [Project overview](docs/project_overview.md) — summary of the scope and technical content.
- [Implementation notes](docs/implementation_notes.md) — mapping from the original files to the repository and implementation caveats.
- [References](docs/references.md) — research works cited in the project.

## MATLAB requirements

- MATLAB. The original Live Scripts were authored with MATLAB R2024a.
- MATLAB's standard ODE solver `ode45`.

No additional toolbox dependency is explicitly identified in the supplied MATLAB scripts.

## Running the simulations

From the repository root, add the source tree to the MATLAB path:

```matlab
addpath(genpath("src"));
```

Then run each controller script separately:

```matlab
run("src/controllers/sliding_mode.m")
run("src/controllers/polar_coordinates.m")
run("src/controllers/dynamic_feedback_linearization.m")
```

Each script clears the workspace, initializes its own simulation parameters, integrates the relevant model with `ode45`, and creates MATLAB figures. The scripts are standalone entry points, not functions to be called from one another.

## Models and controller dependencies

| Script | Model function used | Main purpose |
|---|---|---|
| `sliding_mode.m` | `unicycle_model.m` | Discrete-time sliding-mode control and physical-model trajectory integration |
| `polar_coordinates.m` | `unicycle_model.m` | Regulation using polar-coordinate errors |
| `dynamic_feedback_linearization.m` | `extended_unicycle.m` | Regulation using dynamic feedback linearization |

The standard unicycle model includes the sinusoidal disturbance defined in `unicycle_model.m`. The extended model sets its disturbance to zero, as in the original project script.

## Results

The scripts generate trajectories, state histories and control-input plots interactively in MATLAB. The `results/figures/` directory is reserved for exported figures; it is currently a placeholder and does not contain generated result images.

## Implementation and validation notes

- The original `.mlx` Live Scripts have been exported as ordinary `.m` scripts and placed directly in `src/controllers/`; the two model functions are in `src/models/`.
- The function declaration in `extended_unicycle.m` was aligned with the filename and with the function call in the dynamic-feedback-linearization script.
- The original `controller.m` was an incomplete fragment with undefined variables and is not included as an executable file. `Smooth_TV.mlx` contained no substantive MATLAB code and was omitted.
- The sliding-mode script calculates chained-form discrete updates but advances the stored state using integration of the physical model. This behavior from the source script has been retained and is documented in [implementation notes](docs/implementation_notes.md).
- The code has been organized but has **not been executed in MATLAB as part of this repository review**. Check runtime behavior and numerical results in MATLAB before relying on them.

## Authors

Mario Pintore · Gabriele Tedeschi · Matteo Goddi

Digital Control Systems — Academic Year 2024/2025
