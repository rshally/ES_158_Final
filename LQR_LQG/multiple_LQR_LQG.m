%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Macro-level script to run multiple LQR-LQG trials to get distribution of
% performance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model_type = 'LQG';

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
        cost(nm, nt) = LQR_LQG_model(model_type, 0, [sigmaV, sigmaW]);
        
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



