% Define the function using a symbolic variable
syms x;
f = x^3 - 0.165*x^2 + 3.993e-4; % Enter the function here
g = diff(f); % The derivative of the function

% Predefined values for precision and initial approximation
n = 5; % Number of decimal places
epsilon = 5*10^-(n+1);
x0 = 0.05; % Initial approximation

% Newton-Raphson iteration
iter = 0;
while true
    f0 = vpa(subs(f, x, x0)); % Calculating the value of function at x0
    f0_der = vpa(subs(g, x, x0)); % Calculating the value of function derivative at x0
    x_new = x0 - f0/f0_der; % The Formula
    err = abs(x_new - x0);

    if err < epsilon % Checking the amount of error at each iteration
        break;
    end

    x0 = x_new;
    iter = iter + 1;
    if iter > 100 % Maximum iteration limit
        break;
    end
end

x_new = x_new - rem(x_new, 10^-n); % Displaying up to required decimal places
fprintf('The Root is: %.18f\n', x_new);
fprintf('No. of Iterations: %d\n', iter);

% Plotting the function
x_range = linspace(0, 0.1);
f_vals = x_range.^3 - 0.165*x_range.^2 + 3.993*10^-4;
figure;
plot(x_range, f_vals, 'b', x_range, zeros(size(x_range)), 'r--');
grid on;
title('Graph of the Function');
xlabel('x');
ylabel('f(x)');