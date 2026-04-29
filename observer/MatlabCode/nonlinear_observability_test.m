%% Analysis of Nonlinear Function: f(x,u) -> y
% Mapping: x (R^12) and u (R^4) to y (R^3)
% Analyzing how first 5 elements of x affect y (x6-x12 and u set to 0)

%% Define your nonlinear function here
% REPLACE THIS WITH YOUR ACTUAL FUNCTION
f = @(x, u) maglevSystemMeasurements_fast(x, u);

%% Parameters
num_samples = 10000;  % Number of random samples to test
x_range = [-5, 5];    % Range for x variables
tolerance = 1e-5;     % Tolerance for considering outputs equal

%% Generate random samples for first 5 elements of x
% Set x6-x12 = 0, u1-u4 = 0
X_samples = zeros(num_samples, 12);
X_samples(:, 1:5) = x_range(1) + (x_range(2)-x_range(1)) * rand(num_samples, 5);
% x6-x12 remain 0
U_samples = zeros(num_samples, 4);  % All u = 0

%% Compute outputs
Y_outputs = zeros(num_samples, 3);
for i = 1:num_samples
    Y_outputs(i, :) = f(X_samples(i, :)', U_samples(i, :)');
end

%% Analyze output distribution and uniqueness
fprintf('\n=== ANALYSIS OF f(x,u) ===\n');
fprintf('Analyzing %d samples\n', num_samples);
fprintf('x1-x5 vary in [%.1f, %.1f], x6-x12=0, u1-u4=0\n\n', x_range(1), x_range(2));

%% Find unique outputs and count occurrences
[unique_outputs, ~, ic] = unique(round(Y_outputs/tolerance)*tolerance, 'rows');
output_counts = accumarray(ic, 1);

% Sort by frequency
[output_counts_sorted, sort_idx] = sort(output_counts, 'descend');
unique_outputs_sorted = unique_outputs(sort_idx, :);

%% Display results
fprintf('Total distinct outputs: %d out of %d samples (%.1f%% unique)\n', ...
    length(unique_outputs), num_samples, 100*length(unique_outputs)/num_samples);
fprintf('Average samples per output: %.2f\n\n', num_samples/length(unique_outputs));

fprintf('Top 10 most common outputs:\n');
fprintf('Rank\tCount\tOutput (y1, y2, y3)\n');
for i = 1:min(1, length(unique_outputs))
    fprintf('%d\t%d\t(%.4f, %.4f, %.4f)\n', ...
        i, output_counts_sorted(i), unique_outputs_sorted(i,1), ...
        unique_outputs_sorted(i,2), unique_outputs_sorted(i,3));
end

%% Find which x combinations produce the same output
fprintf('\n=== ANALYSIS OF MULTIPLE INPUTS PRODUCING SAME OUTPUT ===\n');

% Find outputs that appear multiple times
multi_output_indices = find(output_counts > 1);
fprintf('Number of outputs with multiple x inputs: %d\n', length(multi_output_indices));

% Analyze the first few multi-output cases
num_cases_to_analyze = min(5, length(multi_output_indices));
for idx = 1:num_cases_to_analyze
    output_idx = multi_output_indices(idx);
    output_value = unique_outputs(output_idx, :);
    
    % Find all samples with this output
    sample_indices = find(ic == output_idx);
    num_multi = length(sample_indices);
    
    fprintf('\n--- Case %d: Output (%.4f, %.4f, %.4f) appears %d times ---\n', ...
        idx, output_value(1), output_value(2), output_value(3), num_multi);
    
    % Display the x1-x5 combinations
    fprintf('x1-x5 combinations (rows) that produce this output:\n');
    for j = 1:min(10, num_multi)  % Show first 10 combinations
        fprintf('  Sample %d: [%.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
            sample_indices(j), X_samples(sample_indices(j), 1), ...
            X_samples(sample_indices(j), 2), X_samples(sample_indices(j), 3), ...
            X_samples(sample_indices(j), 4), X_samples(sample_indices(j), 5));
    end
    if num_multi > 10
        fprintf('  ... and %d more combinations\n', num_multi - 10);
    end
end

%% Visualize relationships
fprintf('\n=== VISUALIZATION ===\n');

% Create scatter plots for different combinations
figure('Position', [100, 100, 1200, 800]);

% Plot 1: Output space distribution
subplot(2,3,1);
scatter3(Y_outputs(:,1), Y_outputs(:,2), Y_outputs(:,3), 10, 'b', 'filled');
xlabel('y_1'); ylabel('y_2'); zlabel('y_3');
title('Output Space Distribution');
grid on;

% Plot 2-4: Show how x1 affects each output component
subplot(2,3,2);
plot(X_samples(:,1), Y_outputs(:,1), 'r.', 'MarkerSize', 5);
xlabel('x_1'); ylabel('y_1');
title('y_1 vs x_1');
grid on;

subplot(2,3,3);
plot(X_samples(:,2), Y_outputs(:,2), 'g.', 'MarkerSize', 5);
xlabel('x_2'); ylabel('y_2');
title('y_2 vs x_2');
grid on;

subplot(2,3,4);
plot(X_samples(:,3), Y_outputs(:,3), 'b.', 'MarkerSize', 5);
xlabel('x_3'); ylabel('y_3');
title('y_3 vs x_3');
grid on;

% Plot 5: Color-coded by output cluster
subplot(2,3,5);
[~, ~, colors] = unique(ic, 'stable');
scatter3(X_samples(:,1), X_samples(:,2), X_samples(:,3), 10, colors, 'filled');
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
title('x1-x2-x3 Space Colored by Output');
colorbar;
grid on;

% Plot 6: Histogram of output multiplicities
subplot(2,3,6);
histogram(output_counts, 0:max(output_counts));
xlabel('Number of Inputs Producing Same Output');
ylabel('Frequency');
title('Distribution of Output Multiplicities');
grid on;

%% Statistical summary
fprintf('\n=== STATISTICAL SUMMARY ===\n');
fprintf('Unique output ratio: %.2f%%\n', 100*length(unique_outputs)/num_samples);
fprintf('Maximum multiplicity: %d inputs produce the same output\n', max(output_counts));
fprintf('Minimum multiplicity: %d\n', min(output_counts));
fprintf('Standard deviation of multiplicities: %.2f\n', std(output_counts));

%% Sensitivity analysis
fprintf('\n=== SENSITIVITY ANALYSIS ===\n');
% Analyze how sensitive each output component is to each input
sensitivity = zeros(5, 3);  % 5 inputs, 3 outputs

for i = 1:5  % For each of first 5 x's
    for j = 1:3  % For each output component
        sensitivity(i,j) = std(Y_outputs(:,j)) / std(X_samples(:,i));
    end
end

fprintf('Sensitivity (dy/dx) estimation:\n');
fprintf('      y1      y2      y3\n');
for i = 1:5
    fprintf('x%d:  %.4f  %.4f  %.4f\n', i, sensitivity(i,1), sensitivity(i,2), sensitivity(i,3));
end

%% Function definition (REPLACE THIS WITH YOUR ACTUAL FUNCTION)
function y = nonlinear_function(x, u)
    % THIS IS A PLACEHOLDER FUNCTION
    % REPLACE WITH YOUR ACTUAL NONLINEAR FUNCTION
    % Example: y = [x(1)^2 + x(2)*x(3) + u(1);
    %              sin(x(4)) + cos(x(5)) + u(2);
    %              exp(x(1)/10) + x(2)*x(3)*x(4) + u(3)];
    
    % Placeholder: simple polynomial with cross terms
    y = zeros(3, 1);
    
    % Example nonlinear function
    y(1) = x(1)^2 + x(2)*x(3) + sin(x(4)) + cos(x(5)) + u(1);
    y(2) = x(1)*x(2) + x(3)^2 + exp(x(4)/5) + u(2);
    y(3) = x(1)*x(4) + x(2)*x(5) + tanh(x(3)) + u(3);
    
    % Add some nonlinear coupling between x1-x5
    y(1) = y(1) + 0.1*x(1)*x(4)*x(5);
    y(2) = y(2) + 0.05*sin(x(1)*x(2));
    y(3) = y(3) + 0.1*cos(x(3)*x(4));
end