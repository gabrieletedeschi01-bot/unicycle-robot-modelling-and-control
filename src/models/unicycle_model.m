function dydt = unicycle_model(t, state, v1, v2)
% Kinematic unicycle model with sinusoidal disturbance.
max_disturbance = 0.2;
frequency = 2;
disturbance = max_disturbance * sin(2*pi*frequency*t);
dxdt = v1*cos(state(3)) + disturbance;
dydt = v1*sin(state(3)) + disturbance;
dTdt = v2;
dydt = [dxdt; dydt; dTdt];
end