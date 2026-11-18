# Implementation notes

## Source mapping

| Original file | Repository destination | Purpose |
|---|---|---|
| unicycle_model.m | src/models/unicycle_model.m | Three-state kinematic model with sinusoidal disturbance |
| extended_unicycle.m | src/models/extended_unicycle.m | Four-state extended model |
| Sliding_Mode.mlx | src/controllers/sliding_mode/sliding_mode.m | Discrete-time sliding-mode controller |
| Polar_Coords.mlx | src/controllers/polar_coordinates/polar_coordinates.m | Polar-coordinate regulation |
| Dynamic_FL.mlx | src/controllers/dynamic_feedback_linearization/dynamic_feedback_linearization.m | Dynamic feedback linearization |

## Adjustments and exclusions

- The original extended-model file was named extended_unicycle.m, but declared the function extended_unicycle2, while the Live Script calls extended_unicycle. The function declaration has been aligned with the filename and call site; the model equations are otherwise unchanged.
- The Live Scripts have been exported as ordinary MATLAB .m scripts for easier browsing on GitHub.
- The original controller.m is an incomplete fragment: it references undefined variables and does not provide a complete state derivative. It is not included as executable source.
- Smooth_TV.mlx contains no MATLAB code cells and is not included.
- The sliding-mode script computes the chained-form discrete-time state update and separately integrates the physical unicycle model for trajectory visualization. This distinction is preserved.

## Validation status

The scripts have been organized from the supplied archive but have not been executed in MATLAB here. Runtime behavior and numerical results should be checked in MATLAB before considering the code fully validated.