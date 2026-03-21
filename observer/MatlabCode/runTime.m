% Number of times to run the function
n = 10000;

% Store individual runtimes
runtimes = zeros(1, n);

% Warm-up run (optional but recommended to avoid initial overhead)
disp('Performing warm-up run...');
f(zeros(12,1), zeros(4,1));

% Main timing loop
disp(['Running function ' num2str(n) ' times...']);

for i = 1:n
    x = rand(12, 1);
    u = rand(4, 1);

    tic;
    
    f(x, u);
    
    runtimes(i) = toc;
end

% Calculate statistics
average_time = mean(runtimes);
min_time = min(runtimes);
max_time = max(runtimes);
std_time = std(runtimes);

% Display results
fprintf('\n=== Timing Results ===\n');
fprintf('Number of runs: %d\n', n);
fprintf('Average runtime: %.6f seconds\n', average_time);
fprintf('Minimum runtime: %.6f seconds\n', min_time);
fprintf('Maximum runtime: %.6f seconds\n', max_time);