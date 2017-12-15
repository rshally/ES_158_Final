%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Macro level script to optimize the feedback gain constants for the
% proportional model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Run a stochastic simulation to determine good feedback matrices
type = 'circle';
model = 'P';
func_opt = @(k, plotting) P_discrete(k, plotting);

% Turn off plotting
plotting = 0;
fprintf('Optimizing over k1 and k2\n\n')

% Try 100 trials over k1 and k2
ntrials = 50;
k1 = linspace(.5, 5, ntrials);
k2 = linspace(.2, 3, ntrials);
k3 = linspace(1, 10, ntrials);

[k_1, k_2, k_3] = ndgrid(k1,k2,k3);
err_total = zeros(1, numel(k_1));

t1 = tic;
for n = 1:numel(k_1)
    %fprintf('Trial: %d of %d\n', n, numel(k_1))

    k = [k_1(n), k_2(n), k_3(n)];
    err_total(n) = func_opt(k, plotting);
    
    
    t2 = toc(t1);
    est_time = t2/n * (numel(k_1) - n);
    
    if mod(n, 10)==0
        fprintf('Estimated Time Remaining:\n')
        disp(est_time)
    end
end
err_total = reshape(err_total, size(k_1));
fprintf('Complete...Now Plotting\n\n')

%% Plot the optimal results
% Finding Optimal Cost and K Vector:
fprintf('\nOptimal k Vector:\n')
[minC, minIdx] = min(err_total(:));
opt_k = [k_1(minIdx), k_2(minIdx), k_3(minIdx)];
disp(opt_k)
fprintf('Cost:\b')
disp(minC)

% Plotting Along the optimal k_3 value
figure
k3_opt = find(k_3(1,1,:)==k_3(minIdx));
surf(k_1(:,:,k3_opt), k_2(:,:,k3_opt), err_total(:,:,k3_opt))
xlabel('k1')
ylabel('k2')
zlabel('Quadratic Cost Value')
title(sprintf('Cost Function Over K1, K2 for K3=%.2f',k_3(minIdx)))