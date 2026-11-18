% Unicycle regulation using dynamic feedback linearization.
clear; clc;
T_s = 0.05; T_sim = 20; time = 0:T_s:T_sim;
x_d = 0; y_d = 0;
% Final gains used in the original Live Script.
k_p1 = 2; k_p2 = 12; k_d1 = 3; k_d2 = 7;
x0 = [2; 1; pi/4; -0.5]; % [x; y; theta; z]
omega_max = 5; a_max = 2;
states = []; time_states = []; inputs = [];
triangles = x0; current_state = x0;

for k = 1:length(time)-1
    x = current_state(1); y = current_state(2);
    theta = current_state(3); z = current_state(4);
    u1 = -k_p1*x-k_d1*(z*cos(theta));
    u2 = -k_p2*y-k_d2*(z*sin(theta));
    T = [cos(theta),-z*sin(theta);sin(theta),z*cos(theta)];
    res = T\\[u1;u2];
    a = max(min(res(1),a_max),-a_max);
    omega = max(min(res(2),omega_max),-omega_max);
    inputs = [inputs;a,omega];

    t_span = [time(k),time(k+1)];
    [t,x_sol] = ode45(@(t,x) extended_unicycle(t,x,a,omega),t_span,current_state);
    states = [states;x_sol]; time_states = [time_states;t];
    current_state = x_sol(end,:)'; triangles = [triangles,current_state];
end

figure;
plot(states(:,1),states(:,2),'b','LineWidth',1.5); hold on;
plot(x_d,y_d,'ro','MarkerSize',8,'MarkerFaceColor','r');
xlabel('x'); ylabel('y'); title('Planar trajectory with orientation');
grid on; axis equal;

scale = 0.2;
for k = 1:floor(length(time)/20)
    if ~ismember(k,[5 6 7 8 9])
        x = triangles(1,20*k-19); y = triangles(2,20*k-19);
        theta = triangles(3,20*k-19);
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
plot(time_states,states(:,4),'k','LineWidth',1.5);
xlabel('time'); ylabel('values');
legend('x(t)','y(t)','theta(t)','z(t)'); title('Extended unicycle states'); grid on;

input_time = repelem(time(1:end-1),2);
a_step = reshape(repmat(inputs(:,1)',2,1),[],1);
omega_step = reshape(repmat(inputs(:,2)',2,1),[],1);
input_time = [input_time,input_time(end)+T_s];
a_step = [a_step;a_step(end)]; omega_step = [omega_step;omega_step(end)];
figure;
stairs(input_time,a_step,'b','LineWidth',1.5); hold on;
stairs(input_time,omega_step,'r','LineWidth',1.5);
title('Control inputs a and omega'); xlabel('time'); ylabel('values');
legend('a','omega'); grid on;