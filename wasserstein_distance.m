clc; clear; close all;

%% ===================== PARAMETERS =====================
N = 5;                 % number of robots
T = 50;                 % simulation time
dt = 0.001;
steps = round(T/dt);

kv = 1.0;               % linear velocity gain
kw = 2.0;               % angular velocity gain
km = 0.45;               % mean (Wasserstein) gain
ks = 0.245;               % variance (spread) gain

%% Desired distribution (Gaussian)
m_des = [2; 4];                 % desired mean
Sigma_des = 2.0*ones(2);         % desired covariance
sigma_des = sqrt(diag(Sigma_des));

%% ===================== INITIAL CONDITIONS =====================
x = randn(N,1);
y = randn(N,1);
theta = 2*pi*rand(N,1);

%% ===================== STORAGE =====================
x_hist = zeros(steps,N);
y_hist = zeros(steps,N);
v_hist = zeros(steps,N);
w_hist = zeros(steps,N);
W2_hist = zeros(steps,1);
mu_hist = zeros(steps,2);
sigma_hist = zeros(steps,2);
theta_hist = zeros(steps,N);
%% ===================== SIMULATION LOOP =====================
for k = 1:steps

    %% Empirical statistics
    mu = [mean(x); mean(y)];
    sigma = [std(x); std(y)];
    W2_hist(k) = norm(mu - m_des)^2 + norm(sigma - sigma_des)^2;
    mu_hist(k,:) = mu';
    sigma_hist(k,:) = sigma';

    %% ---------- WASSERSTEIN CONTROL TERMS ----------

    % Mean steering term
    u_m = -km * (mu - m_des);

    % Variance (spread) regulation term
    e_sigma = sigma - sigma_des;
    u_s = -ks * e_sigma;

    % Combined desired velocity field (distributional)
    u_des = u_m + u_s;

    theta_ref = atan2(u_des(2), u_des(1));
    v_ref = norm(u_des);

    %% ---------- AGENT-LEVEL CONTROL (UNICYCLE) ----------
    v = kv * v_ref .* cos(theta - theta_ref);
    omega = -kw * sin(theta - theta_ref);

    %% ---------- SYSTEM DYNAMICS ----------
    x = x + dt * v .* cos(theta);
    y = y + dt * v .* sin(theta);
    theta = theta + dt * omega;

    %% ---------- STORE ----------
    x_hist(k,:) = x';
    y_hist(k,:) = y';
    theta_hist(k,:) = theta';
    v_hist(k,:) = v';
    w_hist(k,:) = omega';
end

%% ===================== TIME =====================
t = (0:steps-1)' * dt;

%% ===================== PLOTS =====================

%% 1) Agent trajectories
figure; 
plot(x_hist(:,1), y_hist(:,1),'LineWidth',1.1,'Marker','x','MarkerSize',1.1);
hold on;
for i = 1:N
    plot(x_hist(:,i), y_hist(:,i),'LineWidth',1.1);
end
scatter(m_des(1), m_des(2),120,'r','x','LineWidth',2);
axis equal; grid on;
xlabel('$x$','FontSize',14,'Interpreter','latex'); ylabel('$y$','FontSize',14,'Interpreter','latex');
title('Agent trajectories (Wasserstein mean + variance control)');

%% 2) Agent-wise x_i(t)
figure; hold on;
for i = 1:N
    plot(t, x_hist(:,i),'LineWidth',1.0);
end
yline(m_des(1),'--r','LineWidth',1.5);
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$x_i(t) [m]$','FontSize',14,'Interpreter','latex');
legend('$x_1$','$x_2$','$x_3$','$x_4$','$x_5$','Location','best','Interpreter','latex');
title('Agent-wise x-position evolution');
grid on;

%% 3) Agent-wise y_i(t)
figure; hold on;
for i = 1:N
    plot(t, y_hist(:,i),'LineWidth',1.0);
end
yline(m_des(2),'--r','LineWidth',1.5);
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$y_i(t) [m]$','FontSize',14,'Interpreter','latex');
title('Agent-wise y-position evolution');
legend('$y_1$','$y_2$','$y_3$','$y_4$','$y_5$','Location','best','Interpreter','latex');
grid on;

%% 4) Agent-wise theta_i(t)
figure; hold on;
for i = 1:N
    plot(t,theta_hist(:,i),'LineWidth',1.0);
end
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$\vartheta_i(t) [rad]$','FontSize',14,'Interpreter','latex');
legend('$\vartheta_1$','$\vartheta_2$','$\vartheta_3$','$\vartheta_4$','$\vartheta_5$','Location','best','Interpreter','latex');
title('Agent-wise theta-orientation evolution');
grid on;
%% 4) Mean μ(t)
figure; hold on;
plot(t, mu_hist(:,1),'LineWidth',2);
plot(t, mu_hist(:,2),'LineWidth',2);
yline(m_des(1),'--r','LineWidth',1.5);
yline(m_des(2),'--k','LineWidth',1.5);
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$\mu(t)$','FontSize',14,'Interpreter','latex');
legend('\mu_x','\mu_y','Location','best');
title('Empirical mean evolution');
grid on;

%% 5) Standard deviation σ(t)
figure; hold on;
plot(t, sigma_hist(:,1),'LineWidth',2);
plot(t, sigma_hist(:,2),'LineWidth',2);
yline(sigma_des(1),'--r','LineWidth',1.5);
yline(sigma_des(2),'--k','LineWidth',1.5);
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$\sigma(t)$','FontSize',14,'Interpreter','latex');
legend('\sigma_x','\sigma_y','Location','best');
title('Empirical standard deviation evolution');
grid on;

%% 6) Control inputs v_i(t)
figure; hold on;
for i = 1:N
    plot(t, v_hist(:,i),'LineWidth',1.0);
end
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$v_i(t) [m/sec]$','FontSize',14,'Interpreter','latex');
legend('$u_1$','$u_2$','$u_3$','$u_4$','$u_5$','Location','best','Interpreter','latex');
title('Linear velocity inputs (per agent)');
grid on;

%% 7) Control inputs ω_i(t)
figure; hold on;
for i = 1:N
    plot(t, w_hist(:,i),'LineWidth',1.0);
end
xlabel('$t [s]$','FontSize',14,'Interpreter','latex');
ylabel('$\omega_i(t) [rad/sec]$','FontSize',14,'Interpreter','latex');
legend('$\omega_1$','$\omega_2$','$\omega_3$','$\omega_4$','$\omega_5$','Location','best','Interpreter','latex');
title('Angular velocity inputs (per agent)');
grid on;

figure;
plot(t, W2_hist,'LineWidth',2.5);
xlabel('$t [sec]$','FontSize',14,'Interpreter','latex');
ylabel('$W_2^2(\rho(t),\rho_d)$','FontSize',14,'Interpreter','latex');
title('Wasserstein-2 distance convergence');
grid on;