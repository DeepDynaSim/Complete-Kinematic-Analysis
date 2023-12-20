%% Crank & Connecting Rod Mechanism
%--- Dimensions of mechanism links ---------------------------------------
l2 = 0.5; l3 = 1.5; ldc = 0.1;

%--- Numerical solution for theta2 values from 0 to 360 degrees ----------
%--- Initial position th2 = 60 degrees.
%--- ns = 1 for kinematic analysis at the given position only.
w2 = -10; % Angular velocity in rad/s (Constant)
ns = 201; % Number of samples
tp = 2 * pi / w2; % Time period
dt = tp / (ns - 1); % Time step
t = 0:dt:tp; % Time vector

% Angular velocity vector (negative for clockwise direction)
w2 = -w2 * ones(1, ns); 
acc2 = zeros(1, ns); % Angular acceleration (assumed zero here)

% Angular position of link 2 (th2 in radians)
th2 = deg2rad(60) + w2 .* t;

% Initial guess for the numerical calculations
x = [deg2rad(345), 1.65]; 
xe = 0.001 * abs(x); % Error tolerance
niter1 = 5; niter2 = 20; % Iteration limits
xe = transpose(abs(xe));
kerr = 1; % Error flag

% Main loop for each time step
for k = 1:ns
    for n = 1:niter2
        th3(k) = x(1); X(k) = x(2);
        a = zeros(2, 2); b = zeros(2, 1);

        % Jacobian matrix
        a(1, 1) = -l3 * sin(th3(k)); a(1, 2) = -1;
        a(2, 1) = l3 * cos(th3(k));

        % Function values
        b(1, 1) = l2 * cos(th2(k)) + l3 * cos(th3(k)) - X(k);
        b(2, 1) = l2 * sin(th2(k)) + l3 * sin(th3(k)) - ldc;

        % Newton-Raphson method
        eps = a \ b; % More efficient than inv(a) * b
        x = x - transpose(eps);
        if n > niter1 && all(abs(eps) < xe)
            kerr = 0; break;
        end
    end
    if kerr == 1
        error('Error in iteration');
    end
    th3(k) = x(1); X(k) = x(2);

    % Velocity calculation
    b(1, 1) = l2 * w2(k) * sin(th2(k));
    b(2, 1) = -l2 * w2(k) * cos(th2(k));
    vel = a \ b;
    w3(k) = vel(1); vx1(k) = vel(2);

    % Acceleration calculation
    b(1, 1) = l2 * acc2(k) * sin(th2(k)) + l2 * w2(k)^2 * cos(th2(k)) + l3 * w3(k)^2 * cos(th3(k));
    b(2, 1) = -l2 * acc2(k) * cos(th2(k)) + l2 * w2(k)^2 * sin(th2(k)) + l3 * w3(k)^2 * sin(th3(k));
    acc = a \ b;
    acc3(k) = acc(1); ax1(k) = acc(2);
end

% Kinematic Analysis Data Preparation
VA = [w3; acc3; vx1; ax1]'; % Combine kinematic data

% Display the matrix using fprintf
fprintf('Kinematic Analysis (th2 = 60)\n');
fprintf('%-8s %-8s %-8s %-8s\n', 'w3', 'acc3', 'vx1', 'ax1'); % Header
for i = 1:size(VA, 1)
    fprintf('%-8.4f %-8.4f %-8.4f %-8.4f\n', VA(i, 1), VA(i, 2), VA(i, 3), VA(i, 4));
end

% Initialize arrays for points A, B, and C
xA = zeros(1, ns); yA = zeros(1, ns);
xB = zeros(1, ns); yB = zeros(1, ns);
xC = X; yC = ldc * ones(1, ns); 

%--- Animation of the mechanism's movement -----------------------

% Initialize figure for animation
figure;
axis equal;
xlim([-0.6, 2.2]);
ylim([-0.6, 0.6]);
hold on;
grid on;

%--- Initialize Video Writer ---------------------------------------
writerObj = VideoWriter('mechanism_animation.avi');
open(writerObj);

%--- Initialize figure for animation ------------------------------
figure;
axis equal;
xlim([-0.6, 2.2]);
ylim([-0.6, 0.6]);
hold on;
grid on;

%--- Animation loop ------------------------------------------------
for i = 1:ns
    % Calculate positions of points A, B, C, and F
    xA = 0; yA = 0;
    xB = l2 * cos(th2(i)); yB = l2 * sin(th2(i));
    xC = X(i); yC = ldc;
    xf = l2 * cos(th2(i)) + l3/2 * cos(th3(i));
    yf = l2 * sin(th2(i)) + l3/2 * sin(th3(i));

    % Clear previous plot
    cla;

    % Plot links AB and BC
    plot([xA, xB], [yA, yB], 'k-o', 'LineWidth', 2);
    plot([xB, xC], [yB, yC], 'b-o', 'LineWidth', 2);

    % Plot point F (midpoint of link 3)
    plot(xf, yf, 'r*', 'LineWidth', 1);
    text(xf, yf, ' F');

    % Plot points A, B, C
    text(xA, yA, ' A');
    text(xB, yB, ' B');
    text(xC, yC, ' C');

    % Update the figure window
    drawnow;

    % Capture and write the frame to the video
    frame = getframe(gcf);
    writeVideo(writerObj, frame);

    % Pause for animation effect
    pause(dt);
end

%--- Finalize and Close the Video Writer --------------------------
close(writerObj);

hold off;

% Plotting kinematic properties against th2
%figureTitles = {'\theta_3 (Degree)', 'X (m)', '\omega_3 (rad/s)', 'v_x (m/s)', '\alpha_3 (rad/s^2)', 'a_x (m/s^2)'};
%dataToPlot = {rad2deg(th3), X, w3, vx1, acc3, ax1};
%xLabel = '\theta_2 (Degree)';
%for figIdx = 1:length(dataToPlot)
%    figure(figIdx + 1); % Create new figure
%    plot(rad2deg(th2), dataToPlot{figIdx}, 'LineWidth', 3);
%    xlabel(xLabel, 'FontSize', 14);
%    ylabel(figureTitles{figIdx}, 'FontSize', 14);
%end

