% Four Bar Mechanism

% Velocity and Acceleration Extraction (Symbolically)

%syms th2(t) th3(t) th4(t) l2 l3 l4 lde lae 
%f1=l2*cos(th2)+l3*cos(th3)+l4*cos(th4)-lde==0;
%f2=l2*sin(th2)+l3*sin(th3)+l4*sin(th4)+lae==0;
%fv1=diff2(f1,t);
%fv2=diff2(f2,t);
%fa1=diff2(diff2(f1,t),t);
%fa2=diff2(diff2(f2,t),t);

% Define physical parameters of the mechanism
L2 = 0.3;  % m, length of arm 2
L3 = 0.45; % m, length of arm 3
L4 = 0.7;  % m, length of arm 4
LDE = 0.75; % m
LAE = 0.35; % m

% Define simulation parameters
Nmax = 100; % Maximum number of iterations
x = [20*pi/180, 320*pi/180]; % Adjusted initial guess for th3, th4 (in radians)
xe = 0.0001 * abs(x); % Slightly relaxed error tolerance
dth = 5 * pi / 180; % Step size for th2
th2 = 0:dth:2*pi; % Range of th2 (in radians)
w2 = -0.2944 * ones(size(th2)); % Constant angular velocity w2
acc2 = zeros(size(th2)); % Constant angular acceleration acc2

% Preallocate arrays for results
th3 = zeros(size(th2));
th4 = zeros(size(th2));
w3 = zeros(size(th2));
w4 = zeros(size(th2));
acc3 = zeros(size(th2));
acc4 = zeros(size(th2));

% Main loop for each th2 value
for k = 1:length(th2)
    x_temp = x; % Temporary variable for the current iteration
    kerr = 1; % Flag for convergence

    % Iterative solver for position analysis
    for n = 1:Nmax
        th3_temp = x_temp(1);
        th4_temp = x_temp(2);
        J = [-L3*sin(th3_temp), -L4*sin(th4_temp); L3*cos(th3_temp), L4*cos(th4_temp)];
        
        % Check for singularity in Jacobian
        if cond(J) > 1e10
            disp(['Warning: Jacobian is near singular at th2 index: ', num2str(k)]);
            break;
        end

        f = [-(L2*cos(th2(k)) + L3*cos(th3_temp) + L4*cos(th4_temp) - LDE); ...
             -(L2*sin(th2(k)) + L3*sin(th3_temp) + L4*sin(th4_temp) + LAE)];
        eps = J\f; % More efficient than inv(J)*f
        x_temp = x_temp + eps';
        if all(abs(eps) < xe)
            kerr = 0;
            break;
        end
    end

    if kerr == 1
        disp(['Error: Solution did not converge at th2 index: ', num2str(k), ' (th2 = ', num2str(th2(k) * 180/pi), ' degrees)']);
    else
        th3(k) = x_temp(1);
        th4(k) = x_temp(2);

        % Velocity analysis
        fv = [-L2*sin(th2(k))*w2(k); L2*cos(th2(k))*w2(k)];
        vel = J\fv;
        w3(k) = vel(1);
        w4(k) = vel(2);

        % Acceleration analysis
        fa = [-L2*cos(th2(k))*(w2(k))^2 - L3*cos(th3(k))*(w3(k))^2 - L4*cos(th4(k))*(w4(k))^2 - L2*sin(th2(k))*acc2(k); ...
              L2*cos(th2(k))*acc2(k) - L3*sin(th3(k))*(w3(k))^2 - L4*sin(th4(k))*(w4(k))^2 - L2*sin(th2(k))*(w2(k))^2];
        acc = J\fa;
        acc3(k) = acc(1);
        acc4(k) = acc(2);
    end
end

% Convert angles to degrees for plotting
th2d = th2 * 180/pi;
th3d = th3 * 180/pi;
th4d = th4 * 180/pi;

% Plotting results
figure(1);
subplot(4,3,1), plot(th2d, th3d, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\theta_3 (^o)'), grid on;
subplot(4,3,2), plot(th2d, w3, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\omega_3 (rad/s)'), grid on;
subplot(4,3,3), plot(th2d, acc3, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\alpha_3 (rad/s^2)'), grid on;
subplot(4,3,4), plot(th2d, th4d, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\theta_4 (^o)'), grid on;
subplot(4,3,5), plot(th2d, w4, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\omega_4 (rad/s)'), grid on;
subplot(4,3,6), plot(th2d, acc4, 'r', 'LineWidth', 2), xlabel('\theta_2 (^o)'), ylabel('\alpha_4 (rad/s^2)'), grid on;