% Extended Kalman Filter (EKF) 
% KF: State observer for Discrete systems with assumption of:
% Gaussian/Normal process and measurement noises. - if it is
% normal-standard yes. but for simply normal mean is std =1symmetric
% distribution 
% bell-curve 
% Approximates / linearizes the nonlinear terms in each iteration 
% Nonlinear MSD with Cubic NLY (Nonlinearity) 
% EKF is decent for weak nonlinearity as it tries to linearize the
% nonlinearities around the state points in each iteration using: the
% Jacobian matrices 

clc; clear all; close all; 
% Define the nonlinear cubic mass-spring-damper system parameters
m = 1;        % Mass (kg)
k = 1e4; %20;       % Linear spring constant (N/m)
ccr = 2 * sqrt(k*m); % Critical Damping 
zeta = 0.01; % Damping ratio - 0.01 is lightly-damped system 
c = zeta * ccr %5;        % Damping coefficient (N*s/m = kg/sec)
kNL = 100;%5;      % Nonlinear spring constant (N/m^3)

% Define the time vector and input (force) signal
t = linspace(0, 10, 1e6); %t = 0:0.01:10;         % Time vector (0 to 10 seconds, step 0.01s)
u = ones(size(t));     % Constant input force of 1 N - unit step input starting at t = 0

% Define the nonlinear system dynamics with cubic nonlinearity
f = @(x, u) [x(2); -k/m * x(1) - kNL/m * x(1)^3 - c/m * x(2) + u/m]; % Nonlinear state equations - with inputs: x and u
% u and x as inputs are indeed placeholders for numeric inputs  
% above is: annonymous function 'f' with 'x' and 'u' as inputs - x(2
% represents the second element of vector x) 
% note f is NOT ss but this is First-Order ODEs 
h = @(x) x(1);                                                      % Measurement equation (position)
% The measurement equation is defined as h = @(x) x(1) because it 
% represents that only the position (first state) is measured directly.
% Yes, you can have an observable system even if only one state is directly measured. 
% Observability means you can infer all internal states from the output measurements over finite time. 
% If you can reconstruct the unmeasured state from the measured state and the system dynamics, the system is observable. 

% Simulate the nonlinear system response using ode45
x0 = [0; 0]; % Initial state
[~, x] = ode45(@(t, x) f(x, 1), t, x0); % use tilde ~ to ignore the time vector of solver (in this case)    
% f(x, 1) means the input is: 1 

% Add measurement noise to the output
noise_variance = 1e-20; % 0.1; % artificially contaminating/burrying the response with noise 
y_noisy = x(:, 1) + sqrt(noise_variance) * randn(size(x(:, 1)));

% Plot the true and noisy measurements
figure;
plot(t, x(:, 1), 'b', 'DisplayName', 'True Position');
hold on;
plot(t, y_noisy, 'r', 'DisplayName', 'Noisy Measurement');
xlabel('Time (s)');
ylabel('Position (m)');
legend;
title('Nonlinear Cubic Mass-Spring-Damper System Response');

% Define the Extended Kalman Filter parameters
Q = [0.01 0; 0 0.01];  % Process noise covariance
R = noise_variance;    % Measurement noise covariance
P = eye(2);            % Initial error covariance
x_hat = [0; 0];        % Initial state estimate
% take high Q if you feel less confident on the model and want to trust
% measurement more.
% take high R when you have less confidance on measreuemtn due to e.g.,
% nosiy acc and prefer to trust model more. - not recommended to have both
% Q and R high 

% Preallocate arrays for storing estimates
x_hat_arr = zeros(2, length(t));
P_arr = zeros(2, 2, length(t));

% Extended Kalman filter iteration using 'for' loop 
for k = 1 : length(t)
    % Prediction step - prediction step 
    x_hat = f(x_hat, u(k));            % Predict the state using nonlinear dynamics OR: xhat=A*xhat+B*u(k)
    F = [0 1; -k/m - 3*kNL/m * x_hat(1)^2 -c/m]; % Jacobian of f with respect to x | EKF uses Jacobian to linearize and not good idea for stron/steep slopes
    P = F * P * F' + Q;                % Predict the error covariance
    
    % Measurement update step - correction step 
    H = [1 0];                         % Jacobian of h with respect to x
    K = P * H' / (H * P * H' + R);     % Compute Kalman gain
    x_hat = x_hat + K * (y_noisy(k) - h(x_hat)); % Update state estimate
    P = (eye(2) - K * H) * P;          % Update error covariance
    
    % Store estimates
    x_hat_arr(:, k) = x_hat;
    P_arr(:, :, k) = P;
end

% Plot the estimated states
figure;
plot(t, x_hat_arr(1, :), 'r', 'DisplayName', 'Estimated Position');
hold on;
plot(t, x(:, 1), 'b--', 'DisplayName', 'True Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend;
title('Extended Kalman Filter State Estimation - Position');

figure;
plot(t, x_hat_arr(2, :), 'r', 'DisplayName', 'Estimated Velocity');
hold on;
plot(t, x(:, 2), 'b--', 'DisplayName', 'True Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend;
title('Extended Kalman Filter State Estimation - Velocity');

% accuracy of the state observer
% Calculate RMSE for position and velocity
rmse_position = sqrt(mean((x(:, 1) - x_hat_arr(1, :)').^2));
rmse_velocity = sqrt(mean((x(:, 2) - x_hat_arr(2, :)').^2));

fprintf('RMSE Position: %.4f\n', rmse_position);
fprintf('RMSE Velocity: %.4f\n', rmse_velocity);  
