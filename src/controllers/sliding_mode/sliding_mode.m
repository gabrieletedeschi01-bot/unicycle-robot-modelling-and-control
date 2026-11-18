% Discrete-time sliding-mode controller for the unicycle robot.
clear; clc;
tau = 1; u1_max = 0.5; u2_max = 5;
k_star = [7,5,5];
x = [2;1;pi/4];

% Transform initial condition into chained form: [x; tan(theta); y].
x0 = [x(1);tan(x(3));x(2)];
N_steps = 10;
states = zeros(3,N_steps+1); states(:,1) = x0;
evolution = []; controls = zeros(3,N_steps);
input_vel = []; input_omega = [];

for k = 1:N_steps
    x1 = states(1,k); x2 = states(2,k); x3 = states(3,k);
    s1 = x1-max((k_star(1)-k)/k_star(1),0)*x0(1);
    s2 = x2-max((k_star(2)-k)/k_star(2),0)*x0(2);
    s3 = x3-max((k_star(3)-k)/k_star(3),0)*x0(3);

    if k < k_star(1)
        u1_eq = -s1/tau-x0(1)/(tau*k_star(1));
    else
        u1_eq = -x1/tau;
    end
    u1 = max(min(u1_eq,u1_max),-u1_max);

    if k < k_star(2)
        u21_hat_eq = -s2-x0(2)/k_star(2);
    else
        u21_hat_eq = -x2;
    end
    if k < k_star(3)
        u22_hat_eq = -s3-tau*x2*u1-x0(3)/k_star(3);
    else
        u22_hat_eq = -x3-tau*x2*u1;
    end

    T = [tau/2,tau/2;(3*tau^2*u1)/8,(tau^2*u1)/8];
    T_inv = [(tau^2)/8,-tau/(2*u1);-(3*tau^2)/8,tau/(2*u1)]/(-(tau^3)/8);
    u_2 = T_inv*[u21_hat_eq;u22_hat_eq];
    u21 = max(min(u_2(1),u2_max),-u2_max);
    u22 = max(min(u_2(2),u2_max),-u2_max);

    if abs(x1)<=1e-4 && abs(x2)<=1e-4 && abs(x3)<=1e-4
        u1=0; u21=0; u22=0;
    end

    % The discrete chained-form update is computed in the original script,
    % while the stored state is advanced using the physical-model integration.
    x1_next = x1+tau*u1; %#ok<NASGU>
    x2_next = x2+(tau/2)*u21+(tau/2)*u22; %#ok<NASGU>
    x3_next = x3+tau*x2*u1+(3*tau^2/8)*u21*u1+(tau^2/8)*u22*u1; %#ok<NASGU>

    % Physical unicycle integration over the two half-sampling intervals.
    v1 = u1/cos(atan(x2));
    v21 = u21/sec(atan(x2))^2;
    v22 = u22/sec(atan(x2))^2;
    t_span = [0,tau/2];
    init_state = [x1;x3;atan(x2)];
    [~,sol] = ode45(@(t,state) unicycle_model(t,state,v1,v21),t_span,init_state);
    evolution = [evolution,sol'];
    init_state = sol(end,:)';
    [~,sol] = ode45(@(t,state) unicycle_model(t,state,v1,v22),t_span,init_state);
    evolution = [evolution,sol'];
    states(:,k+1) = sol(end,:)';
    controls(:,k) = [u1;u21;u22];
    input_vel = [input_vel;v1;v1];
    input_omega = [input_omega;v21;v22];
end

x_real = states(1,:); y_real = states(3,:); theta_real = atan(states(2,:));
figure;
plot(0:N_steps,x_real,'b','LineWidth',1.5); hold on;
plot(0:N_steps,y_real,'g','LineWidth',1.5);
plot(0:N_steps,theta_real,'r','LineWidth',1.5);
title('Unicycle states'); xlabel('time'); ylabel('values');
legend('x','y','theta'); grid on;

figure;
plot(0:N_steps,states(1,:),'b','LineWidth',1.5); hold on;
plot(0:N_steps,states(2,:),'g','LineWidth',1.5);
plot(0:N_steps,states(3,:),'r','LineWidth',1.5);
title('Chained-form states'); xlabel('step'); ylabel('values');
legend('x1','x2','x3'); grid on;

figure;
stairs(1:N_steps,controls(1,:),'b','LineWidth',1.5); hold on;
stairs(1:N_steps,controls(2,:),'g','LineWidth',1.5);
stairs(1:N_steps,controls(3,:),'r','LineWidth',1.5);
title('Sliding-mode inputs'); xlabel('time'); ylabel('control inputs');
legend('u1','u21','u22'); grid on;

time_steps = 0:0.5:N_steps-0.5;
figure;
stairs(time_steps,input_vel,'b','LineWidth',1.5); hold on;
stairs(time_steps,input_omega,'r','LineWidth',1.5);
title('Unicycle inputs'); xlabel('time'); ylabel('input');
legend('v','omega'); grid on;

scale = 0.1;
figure;
plot(evolution(1,:),evolution(2,:),'b','LineWidth',1.5); hold on;
title('Planar trajectory with orientation'); xlabel('x'); ylabel('y');
grid on; axis equal;
for k = 1:floor((N_steps+2)/2)
    x_pos = states(1,k); y_pos = states(3,k); theta = atan(states(2,k));
    tri_local = scale*[0.5,-0.5,-0.5;0,0.2,-0.2];
    R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
    tri_global = R*tri_local;
    tri_global(1,:) = tri_global(1,:)+x_pos;
    tri_global(2,:) = tri_global(2,:)+y_pos;
    fill(tri_global(1,:),tri_global(2,:),'k','EdgeColor','none');
end
hold off;