# Project overview

## Scope and objective

The project *Discrete Time Sliding Mode Control Design for Unicycle Robot With Bounded Inputs* was developed for the Digital Control Systems course in academic year 2024/2025 by Mario Pintore, Gabriele Tedeschi and Matteo Goddi.

Its objective is to study regulation of a unicycle mobile robot to the origin in the presence of actuator saturation. The main approach is a discrete-time sliding-mode controller designed from a discrete-time chained-form model. Two continuous-time alternatives are implemented and compared through simulation.

## Unicycle model

The robot configuration is described by \([x,y,\theta]\), with linear velocity \(v\) and angular velocity \(\omega\):

- \(\dot{x}=v\cos(\theta)\)
- \(\dot{y}=v\sin(\theta)\)
- \(\dot{\theta}=\omega\)

The report derives a chained-form representation using the transformation \(x_1=x\), \(x_2=\tan(\theta)\), \(x_3=y\), with transformed inputs. The discretization is developed using Taylor expansions and multirate input sampling.

## Control approaches

### Discrete-time sliding-mode control

The main controller is designed directly for the discrete-time chained-form model. Sliding surfaces are defined to drive the state to the origin in a finite number of steps. The analysis addresses bounded inputs, actuator saturation, the domain of attraction (DOA), and a perturbed-unicycle case in which a disturbance affects the translational velocities.

### Polar-coordinate regulation

The first alternative controller uses the polar error variables \(\rho\), \(\gamma\) and \(\delta\). It is continuous in nature and is simulated with sampled control inputs held over each integration interval.

### Dynamic feedback linearization

The second alternative controller uses a dynamically extended unicycle state, adding the state \(z\). The control law is obtained through feedback linearization and is likewise simulated with sample-and-hold inputs.

## Simulation study and conclusions

The report presents trajectories, state evolutions and control inputs for all three approaches. Its concluding comparison describes the discrete-time sliding-mode controller as fast and suitable for larger sampling periods, while noting the tuning and singularity considerations of polar-coordinate regulation and the higher computational cost and tuning complexity of dynamic feedback linearization.

These are the conclusions reported by the project authors; the MATLAB scripts in this repository have not been independently rerun during this review.

## Included project documents

- [Digital_Control_Systems.pdf](Digital_Control_Systems.pdf) — written report. It covers the model and discretization, sliding-mode design, input saturation and DOA, perturbed unicycle, the two alternative controllers, simulation results, and conclusions.
- [SlideDigital.pdf](SlideDigital.pdf) — presentation summarizing the motivation, goals, models, controller designs, simulation results and comparison.

## Code organization

The exported MATLAB scripts are organized by role:

- `src/models/`: the standard kinematic unicycle model and the extended model.
- `src/controllers/`: the sliding-mode, polar-coordinate and dynamic-feedback-linearization simulations.

See [implementation_notes.md](implementation_notes.md) for the source-file mapping and caveats, and [references.md](references.md) for the cited literature.
