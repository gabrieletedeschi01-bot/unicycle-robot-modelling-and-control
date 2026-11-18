# Unicycle Robot: Modelling and Control

MATLAB project developed for the **Digital Control Systems** course (A.Y. 2024/2025). The work studies stabilization of a unicycle mobile robot and compares a discrete-time sliding-mode controller with two continuous-time alternatives.

## Control approaches

1. **Discrete-time sliding-mode control** — the unicycle kinematics are transformed into chained form and discretized using multirate input sampling. The controller considers bounded inputs, actuator saturation, and finite-step convergence.
2. **Polar-coordinate regulation** — a continuous-time unicycle model is controlled using the polar error coordinates rho, gamma, and delta.
3. **Dynamic feedback linearization** — an extended unicycle model is used to design a regulation controller through dynamic feedback linearization.

The MATLAB scripts generate planar trajectories, state histories, and control-input plots. The written project report also discusses the domain of attraction and a perturbed-unicycle case.

## Repository structure

```text
.
├── src/
│   ├── models/
│   │   ├── unicycle_model.m
│   │   └── extended_unicycle.m
│   └── controllers/
│       ├── sliding_mode/sliding_mode.m
│       ├── polar_coordinates/polar_coordinates.m
│       └── dynamic_feedback_linearization/
│           └── dynamic_feedback_linearization.m
├── docs/
│   ├── project_overview.md
│   ├── implementation_notes.md
│   └── references.md
└── results/
    └── figures/
```

## Requirements

- MATLAB (the original Live Scripts were authored with MATLAB R2024a).
- MATLAB's standard numerical ODE solver, ode45.

No additional toolbox dependency is explicitly identified in the supplied scripts.

## Running the simulations

Open MATLAB at the repository root and add the source folders to the path:

```matlab
addpath(genpath("src"));
```

Run each script separately:

- `src/controllers/sliding_mode/sliding_mode.m`
- `src/controllers/polar_coordinates/polar_coordinates.m`
- `src/controllers/dynamic_feedback_linearization/dynamic_feedback_linearization.m`

Each script initializes its own parameters and generates plots.

## Source and validation notes

- The original MATLAB Live Scripts have been exported as ordinary .m files for easier browsing on GitHub.
- The extended model's function declaration was aligned with its filename and with the call made by the original Live Script.
- The supplied controller.m was an incomplete fragment with undefined variables, so it is documented but not included as executable code. Smooth_TV.mlx contained no MATLAB code cells and was omitted.
- The original sliding-mode script computes chained-form discrete updates but advances its stored state using the integrated physical model. This behavior is retained and described in the implementation notes.
- The original report and presentation remain in the source archive and are summarized in docs/project_overview.md. Third-party books and papers are not redistributed; see docs/references.md.
- The MATLAB scripts have not been executed in this environment. Validate runtime behavior and numerical results in MATLAB before relying on them.

## Authors

Mario Pintore · Gabriele Tedeschi · Matteo Goddi

Digital Control Systems — Academic Year 2024/2025
