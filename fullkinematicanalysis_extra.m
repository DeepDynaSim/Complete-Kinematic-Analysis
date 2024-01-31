% Defining the physical parameters of the mechanism in meters
AB = 20;  % Length of arm AB
BC = 60;  % Length of arm BC
CD = 40;  % Length of arm CD
DE = 40;  % Length of arm DE
CE = 80;  % Length of arm CE
EF = 50;  % Length of arm EF
D_x = 69; % X-coordinate of point D
D_y = 54.5; % Y-coordinate of point D
h = 39.5;  % Height parameter

% Defining the maximum number of iterations for the solver
Nmax = 1000;

% Initial guess values for theta2, theta3, theta4, and s1 (in radians)
x = [210.05 * pi / 180, -96.038 * pi / 180, 179.68 * pi / 180, -14.793];

% Defining the error tolerance relative to the initial guess
xe = 0.00001 * abs(x);

% System inputs: Angular position, velocity and acceleration of theta1
dth = 2 * pi / 360; % Increment for theta1
theta1 = 130 * pi / 180 : dth : 260 * pi / 180; % Range of theta1
w_theta1 = 10.4719755 * ones(1, length(theta1)); % Constant angular velocity of theta1 (100 RPM)
acc_theta1 = zeros(1, length(theta1)); % Constant angular acceleration of theta1 (0)

% Transposing the error tolerance for calculation convenience
xe = transpose(abs(xe));

% Initializing convergence check
kerr = 1; % If kerr = 1, results have not converged

% Main loop over the range of theta1
for t = 1:length(theta1)

    % Iterative solver loop
    for n = 1:Nmax
        % Assigning the initial guess values to the unknowns
        theta2(t) = x(1);
        theta3(t) = x(2);
        theta4(t) = x(3);
        s1(t) = x(4);

        % Constructing the Jacobian matrix
        J = zeros(4, 4);
        J(1, 1) = BC * cos(theta2(t));
        J(1, 2) = CD * cos(theta3(t));
        % Rest of the Jacobian matrix is zero for the first row
        J(2, 1) = -BC * sin(theta2(t));
        J(2, 2) = -CD * sin(theta3(t));
        % Rest of the Jacobian matrix is zero for the second row
        J(3, 2) = DE * cos(theta3(t));
        J(3, 3) = EF * cos(theta4(t));
        % Rest of the Jacobian matrix is zero for the third row
        J(4, 2) = -DE * sin(theta3(t));
        J(4, 3) = -EF * sin(theta4(t));
        J(4, 4) = 1;

        % Defining the function f for the solver
        f = zeros(4, 1);
        f(1) = -(AB * sin(theta1(t)) + BC * sin(theta2(t)) + CD * sin(theta3(t)) + D_y);
        f(2) = -(AB * cos(theta1(t)) + BC * cos(theta2(t)) + CD * cos(theta3(t)) + D_x);
        f(3) = -(DE * sin(theta3(t)) + EF * sin(theta4(t)) + h);
        f(4) = -(DE * cos(theta3(t)) + EF * cos(theta4(t)) + D_x + s1(t));

        % Solving for the increments
        eps = inv(J) * f;
        x = x + transpose(eps);

        % Check if the solution has converged
        if abs(eps) < xe
            kerr = 0;
            break;
        end
    end

    % Error message if the solution has not converged
    if kerr == 1
        disp('Error: Solution has not converged');
    end

    % Updating the solution for the current iteration
    theta2(t) = x(1);
    theta3(t) = x(2);
    theta4(t) = x(3);
    s(t) = x(4);

    % Velocity Analysis
    fv = zeros(4, 1);
    fv(1) = -(AB * cos(theta1(t)) * w_theta1(t));
    fv(2) = -(-AB * sin(theta1(t)) * w_theta1(t));
    % fv(3) and fv(4) remain zero

    % Calculating the velocity
    vel = inv(J) * fv;
    w_theta2(t) = vel(1);
    w_theta3(t) = vel(2);
    w_theta4(t) = vel(3);
    w_s1(t) = vel(4);

    % Acceleration Analysis
    fa = zeros(4, 1);
    fa(1) = -(AB * cos(theta1(t)) * acc_theta1(t) - BC * sin(theta2(t)) * w_theta2(t)^2 - CD * sin(theta3(t)) * w_theta3(t)^2 - AB * sin(theta1(t)) * w_theta1(t)^2);
    fa(2) = -(-AB * cos(theta1(t)) * w_theta1(t)^2 - BC * cos(theta2(t)) * w_theta2(t)^2 - CD * cos(theta3(t)) * w_theta3(t)^2 - AB * sin(theta1(t)) * acc_theta1(t));
    fa(3) = -(-EF * sin(theta4(t)) * w_theta4(t)^2 - DE * sin(theta3(t)) * w_theta3(t)^2);
    fa(4) = -(-EF * cos(theta4(t)) * w_theta4(t)^2 - DE * cos(theta3(t)) * w_theta3(t)^2);

    % Calculating the acceleration
    acc = inv(J) * fa;
    acc_theta2(t) = acc(1);
    acc_theta3(t) = acc(2);
    acc_theta4(t) = acc(3);
    acc_s1(t) = acc(4);
end

% Converting angles from radians to degrees for plotting
theta1d = theta1 * 180 / pi;
theta2d = theta2 * 180 / pi;
theta3d = theta3 * 180 / pi;
theta4d = theta4 * 180 / pi;

% Plotting the results
figure(1);
% Plotting theta2 vs theta1
subplot(4, 3, 1), plot(theta1d, theta2d, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\theta_2(^o)'), grid on;
% Plotting angular velocity of theta2 vs theta1
subplot(4, 3, 2), plot(theta1d, w_theta2, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\omega_2(r/s)'), grid on;
% Plotting angular acceleration of theta2 vs theta1
subplot(4, 3, 3), plot(theta1d, acc_theta2, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\alpha_2(r/s^2)'), grid on;
% Plotting theta3 vs theta1
subplot(4, 3, 4), plot(theta1d, theta3d, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\theta_3(^o)'), grid on;
% Plotting angular velocity of theta3 vs theta1
subplot(4, 3, 5), plot(theta1d, w_theta3, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\omega_3 (r/s)'), grid on;
% Plotting angular acceleration of theta3 vs theta1
subplot(4, 3, 6), plot(theta1d, acc_theta3, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\alpha_3(r/s^2)'), grid on;
% Plotting theta4 vs theta1
subplot(4, 3, 7), plot(theta1d, theta4d, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\theta_4(^o)'), grid on;
% Plotting angular velocity of theta4 vs theta1
subplot(4, 3, 8), plot(theta1d, w_theta4, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\omega_4 (r/s)'), grid on;
% Plotting angular acceleration of theta4 vs theta1
subplot(4, 3, 9), plot(theta1d, acc_theta4, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('\alpha_4(r/s^2)'), grid on;
% Plotting s vs theta1
subplot(4, 3, 10), plot(theta1d, s, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('s_ 1(m)'), grid on;
% Plotting velocity of s vs theta1
subplot(4, 3, 11), plot(theta1d, w_s1, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('V_s _ 1(m/s)'), grid on;
% Plotting acceleration of s vs theta1
subplot(4, 3, 12), plot(theta1d, acc_s1, 'r', 'linewidth', 2), xlabel('\theta_1 (^o)'), ylabel('a_s _ 1(m/s^2)'), grid on;
