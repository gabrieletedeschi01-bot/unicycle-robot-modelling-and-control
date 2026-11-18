# Implementation notes

## Repository layout

The repository separates model functions from controller/simulation scripts. There are no controller-specific subdirectories: all three controller scripts are stored directly in `src/controllers/`.

| Repository file | Original source | Role |
|---|---|---|
| `src/models/unicycle_model.m` | `unicycle_model.m` | Three-state kinematic model with sinusoidal disturbance in the translational dynamics |
| `src/models/extended_unicycle.m` | `extended_unicycle.m` | Four-state dynamically extended unicycle model |
| `src/controllers/sliding_mode.m` | `Sliding_Mode.mlx` | Discrete-time sliding-mode controller in chained coordinates |
| `src/controllers/polar_coordinates.m` | `Polar_Coords.mlx` | Regulation using polar-coordinate errors |
| `src/controllers/dynamic_feedback_linearization.m` | `Dynamic_FL.mlx` | Dynamic feedback linearization |

The original report and presentation are included in `docs/` as `Digital_Control_Systems.pdf` and `SlideDigital.pdf`, respectively.

## Model details

### Standard unicycle model

`unicycle_model.m` implements the kinematic equations and adds the sinusoidal disturbance
`0.2*sin(2*pi*2*t)` to both translational velocity components. The angular dynamics are given by the supplied angular-rate input.

### Extended unicycle model

`extended_unicycle.m` uses the state vector `[x; y; theta; z]`. The translational speed is the extended state `z`, the heading rate is `omega`, and the derivative of `z` is the input `a`. The disturbance is set to zero in the original implementation.

## Controller scripts

- **Sliding mode:** uses a sampling period `tau = 1`, bounded transformed inputs, a finite number of control steps and two half-period integrations of the physical unicycle model.
- **Polar coordinates:** uses `T_s = 0.05`, a 20-second simulation horizon, polar error variables and saturated velocity/angular-velocity commands. Each control command is held over an `ode45` integration interval.
- **Dynamic feedback linearization:** uses the extended state, proportional/derivative gains, saturated acceleration and angular velocity, and `ode45` integration over sample intervals.

All scripts are written as executable MATLAB scripts and initialize their own variables. Run them separately from the repository root after adding `src/` (including its subfolders) to the MATLAB path.

## Adjustments and exclusions

- The original Live Scripts were exported to ordinary `.m` files and placed directly in `src/controllers/`.
- The original `extended_unicycle.m` declared a function named `extended_unicycle2`, while the controller calls `extended_unicycle`. The repository version aligns the declaration with the filename and call site; the model equations are otherwise retained.
- The supplied `controller.m` is an incomplete fragment that references undefined variables and does not implement a complete state derivative. It is therefore not included.
- `Smooth_TV.mlx` contains no substantive MATLAB code and is not included.
- The project report and presentation are included under `docs/`. Third-party papers and textbooks from the original archive are not copied into the repository.

## Important implementation caveat

In `sliding_mode.m`, the script computes the chained-form discrete updates `x1_next`, `x2_next` and `x3_next`, but the next stored state is assigned from the final state returned by integrating the physical unicycle model. The calculated discrete updates are not used for that assignment. This behavior has been preserved from the source script rather than silently changing the implementation.

## Validation status

The repository has been inspected for file organization, source mapping and consistency of documentation. The MATLAB scripts have **not** been run in MATLAB during this review. In particular, this documentation review should not be interpreted as numerical or runtime validation.
