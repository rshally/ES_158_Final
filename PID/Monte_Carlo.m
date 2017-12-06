% Run a stochastic simulation to determine good feedback matrices
type = 'y=sin(t)';
func_opt = @(type, kp_v, kp_th) P_model_V2(type, kp_v, kp_th);

% Turn off plotting
set(0,'DefaultFigureVisible','off')

% Try 100 trials
ntrials = 100;
err_total = zeros(1,ntrials);
load(['best_Pmodel_',type])
for n = 1:ntrials
    fprintf('Trial:\n')
    disp(n)
    kp_v = [2*rand, 2*rand, 1*rand];
    kp_th = [1*rand, 1*rand, 1*rand];
    err_total(n) = func_opt(type, kp_v, kp_th);
    
    % Check if error is now better
    if err_total(n)<err_best
        err_best = err_total;
        kp_v_best = kp_v;
        kp_th_best = kp_th;
        % Save best error and inputs
        fprintf('Saving new best!\n')
        save(['best_Pmodel_',type], 'err_best', 'kp_v_best', 'kp_th_best')
    end
end


%%%% NOTE: Need to time out if taking too long


