% Four Bar Mechanism: Position Analysis

% Inputs
L2 = 0.3; % m, length of arm 2
L3 = 0.45; % m, length of arm 3
L4 = 0.7; % m, length of arm 4
LDE = 0.75; % m
LAE = 0.35; % m
Nmax = 100; % max number of iterations
epsk = [1e-6, 1e-6]; % convergence threshold

% Initialize variables
N = 361; % Number of theta2 values
teta2 = linspace(0, 2*pi, N); % theta2 in radians
teta3 = zeros(1, N); % theta3 in radians
teta4 = zeros(1, N); % theta4 in radians
tet3 = zeros(1, N); % theta3 in degrees
tet4 = zeros(1, N); % theta4 in degrees

% Display progress bar
wbar = waitbar(0, 'Iteration Processing....');

% Main loop
for k = 1:N
    tet3o = 20 * pi / 180; % Reset initial value of tet3o
    tet4o = 320 * pi / 180; % Reset initial value of tet4o
    
    for iter = 1:Nmax
        % Update theta3 and theta4
        tet3o = tet3o + eps(1);
        tet4o = tet4o + eps(2);

        % Define function and Jacobian
        f = [-(L2 * cos(teta2(k)) + L3 * cos(tet3o) + L4 * cos(tet4o) - LDE); -(L2 * sin(teta2(k)) + L3 * sin(tet3o) + L4 * sin(tet4o) + LAE)];
        J = [-L3 * sin(tet3o), -L4 * sin(tet4o); L3 * cos(tet3o), L4 * cos(tet4o)];

        % Compute error and check for convergence
        eps = J \ f; % More efficient than inv(J) * f
        if all(abs(eps) <= epsk)
            tet3(k) = tet3o * 180 / pi; % deg
            teta3(k) = tet3o; % rad
            teta4(k) = tet4o; % rad
            tet4(k) = tet4o * 180 / pi; % deg
            break;
        end
    end
    waitbar(k/N, wbar);
end

% Close progress bar
close(wbar);

% Write results to file
fid = fopen('reportfourarm.m', 'w');
fprintf(fid, 'teta2(i) teta3(i) teta4(i)\n');
for i = 1:N
    fprintf(fid, '%1.2f     %6.1f        %6.1f\n', teta2(i) * 180/pi, tet3(i), tet4(i));
end
fclose(fid);

% Prompt for analysis results
input('Analysis Results: Report | <enter>\n');

% Plot the trajectory of theta_3 and theta_4
plot(teta2 * 180/pi, tet3, 'r-', teta2 * 180/pi, tet4, 'b-');
xlabel('Theta_2 (deg)');
ylabel('Theta_3, Theta_4 (deg)');
legend('Theta_3', 'Theta_4');
title('Trajectory of Theta_3 and Theta_4 vs Theta_2');