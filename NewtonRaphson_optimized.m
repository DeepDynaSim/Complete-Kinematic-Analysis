% Main script to execute Newton-Raphson method
fn = @(v) [v(1)^2+v(2)^2-2*v(3); v(1)^2+v(3)^2-(1/3); v(1)^2+v(2)^2+v(3)^2-1];
jacob_fn = @(v) [2*v(1) 2*v(2) -2; 2*v(1) 0 2*v(3); 2*v(1) 2*v(2) 2*v(3)];
tolerance = 10^-5;
v = [1; 1; 0.1];
max_iterations = 20;

[v_final, iterations, final_error] = NewtonRaphson_nl(v, fn, jacob_fn, max_iterations, tolerance);
fprintf('Final Result: v = [%f, %f, %f], Iterations: %d, Error: %e\n', v_final, iterations, final_error);

function [v_final, num_iterations, final_norm] = NewtonRaphson_nl(v, fn, jacob_fn, max_iterations, tolerance)
    if nargin < 5, tolerance = 10^-5; end
    if nargin < 4, max_iterations = 20; end

    v_final = v;
    f_val = fn(v_final);
    num_iterations = 0;

    while true
        jacobian_val = jacob_fn(v_final);
        delta = jacobian_val \ f_val;
        v_final = v_final - delta;
        f_val = fn(v_final);
        num_iterations = num_iterations + 1;
        final_norm = norm(f_val);

        if final_norm < tolerance || num_iterations >= max_iterations
            break;
        end
    end
end