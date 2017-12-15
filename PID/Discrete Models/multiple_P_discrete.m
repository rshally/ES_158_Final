%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Macro-level script to run multiple proportional trials to get distribution of
% performance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The Model
model_type = 'P';
k_opt = [1.14, 0.43, 3.02];

% Set up number of trials for each
ntrials = 20;
nnoise = 25;
cost = zeros(nnoise, ntrials);

% Baseline noise standard deviations
sigmaV_base = .005;
sigmaW_base = .05;

% Run a loop over each noise factor
t1 = tic;
noise_vec = linspace(0, 5, nnoise);
for nm = 1:nnoise
    noise_mult = noise_vec(nm);
    
    sigmaV = sigmaV_base * noise_mult;
    sigmaW = sigmaW_base * noise_mult;
    
    % Run a loop over the total number of trials
    for nt = 1:ntrials
        cost(nm, nt) = P_discrete(k_opt, 0, sigmaV);
        
        % Print time remaining
        t2 = toc(t1);
        n_elpsd = (nm-1)*ntrials + nt;
        est_time = t2/n_elpsd * (nnoise*ntrials - n_elpsd);
        if mod(n_elpsd, 5)==0
            fprintf('Estimated Time Remaining:\n')
            disp(est_time)
        end
    end
end


% Save the results to a file
save([model_type,'_cost'], cost, noise_vec)

% Plot output: x-axis - multiplicative factor, y-axis - mean value with
figure
errorbar(noise_vec, mean(cost, 2), std(cost,0,2));
xlabel('Noise Multiplicative Factor')
ylabel('Cost Function')
title([model_type,' Cost With Noise Amplification'])


% Compare the LQR and LQG mean values:
if exist('../../LQR_LQG/LQR_cost.mat', 'file') && exist('../../LQR_LQG/LQG_cost.mat', 'file') &&...
   exist('P_cost.mat', 'file')
    % Load saved data
    P_cost = load('P_cost.mat');
    LQR_cost = load('../../LQR_LQG/LQR_cost.mat');
    LQG_cost = load('../../LQR_LQG/LQG_cost.mat');
    
    % Plot noise results
    figure
    errorbar(P_cost.noise_vec, mean(P_cost.cost, 2), std(P_cost.cost,0,2) )
    hold on
    errorbar(LQR_cost.noise_vec, mean(LQR_cost.cost, 2), std(LQR_cost.cost,0,2) )
    hold on
    errorbar(LQG_cost.noise_vec, mean(LQG_cost.cost, 2), std(LQG_cost.cost,0,2) )
    xlabel('Noise Multiplicative Factor')
    ylabel('Cost Functions')
    title(sprintf('Cost Mean Values With Noise'))
    legend('Proportional', 'LQR', 'LQG')
else
    warning('One of the data files does not exist...Find it to show noise results!')
end



