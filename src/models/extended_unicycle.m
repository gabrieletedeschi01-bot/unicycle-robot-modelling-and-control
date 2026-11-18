function dydt = extended_unicycle(~, state, a, omega)
% Four-state extended unicycle model used by dynamic feedback linearization.
% state = [x; y; theta; z]
disturbance = 0; % As set in the original project script.
dxdt = state(4)*cos(state(3)) + disturbance;
dydt = state(4)*sin(state(3)) + disturbance;
dTdt = omega;
dZdt = a;
dydt = [dxdt; dydt; dTdt; dZdt];
end