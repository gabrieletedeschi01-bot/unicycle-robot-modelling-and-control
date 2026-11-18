% Unicycle regulation using polar-coordinate errors.
clear; clc;
T_s = 0.05; T_sim = 20; time = 0:T_s:T_sim;
x_d = 0; y_d = 0;
k1 = 0.8; k2 = 1.8; k3 = 0.95;
x0 = [2; 1; pi/4];
v_max = 0.5; omega_max = 5;
states = []; time_states = []; polar_coords = []; inputs = [];
triangles = x0; current_state = x0;

for k = 1:length(time)-1
    x = current_state(1); y = current_state(2); theta = current_state(3);
    rho = sqrt((x_d-x)^2 + (y_d-y)^2);
    gamma = atan2(y-y_d, x-x_d) - theta + pi;
    delta = theta + gamma;
    polar_coords = [polar_coords; rho, gamma, delta];

    v = max(min(k1*rho*cos(gamma),v_max),-v_max);
    if abs(gamma) <= 1e-2
        omega = k2*gamma + k1*cos(gamma)*(gamma+k3*delta);
    else
        omega = k2*gamma + k1*sin(gamma)*cos(gamma)*(gamma+k3*delta)/gamma;
    end
    omega = max(min(omega,omega_max),-omega_max);
    inputs = [inputs; v, omega];

    t_span = [time(k),time(k+1)];
    [t,x_sol] = ode45(@(t,x) unicycle_model(t,x,v,omega),t_span,current_state);
    states = [states; x_sol]; time_states = [time_states; t];
    current_state = x_sol(end,:)'; triangles = [triangles,current_state];
end

figure;
plot(states(:,1),states(:,2),'b','LineWidth',1.5); hold on;
plot(x_d,y_d,'ro','MarkerSize',8,'MarkerFaceColor','r');
xlabel('x'); ylabel('y'); title('Planar trajectory with orientation');
grid on; axis equal;

scale = 0.2;
for k = 1:floor(length(time)/40)
    if ~ismember(k,[5 6 7 8 9])
        x = triangles(1,40*k-39); y = triangles(2,40*k-39);
        theta = triangles(3,40*k-39);
        tri_local = scale*[0.5,-0.5,-0.5;0,0.2,-0.2];
        R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
        tri_global = R*tri_local;
        tri_global(1,:) = tri_global(1,:)+x;
        tri_global(2,:) = tri_global(2,:)+y;
        fill(tri_global(1,:),tri_global(2,:),'k','EdgeColor','none');
    end
end

figure;
plot(time_states,states(:,1),'r','LineWidth',1.5); hold on;
plot(time_states,states(:,2),'g','LineWidth',1.5);
plot(time_states,states(:,3),'b','LineWidth',1.5);
xlabel('time'); ylabel('values'); legend('x(t)','y(t)','theta(t)');
title('Unicycle states'); grid on;

figure;
plot(time(1:end-1),polar_coords(:,1),'m','LineWidth',1.5); hold on;
plot(time(1:end-1),polar_coords(:,2),'c','LineWidth',1.5);
plot(time(1:end-1),polar_coords(:,3),'k','LineWidth',1.5);
xlabel('time'); ylabel('values'); legend('rho(t)','gamma(t)','delta(t)');
title('Polar coordinates'); grid on;

input_time = repelem(time(1:end-1),2);
v_step = reshape(repmat(inputs(:,1)',2,1),[],1);
omega_step = reshape(repmat(inputs(:,2)',2,1),[],1);
input_time = [input_time,input_time(end)+T_s];
v_step = [v_step;v_step(end)]; omega_step = [omega_step;omega_step(end)];
figure;
stairs(input_time,v_step,'b','LineWidth',1.5); hold on;
stairs(input_time,omega_step,'r','LineWidth',1.5);
xlabel('time'); ylabel('input'); legend('v(t)','omega(t)');
title('Control inputs'); grid on;