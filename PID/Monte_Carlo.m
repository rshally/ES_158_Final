% Run a stochastic simulation to determine good feedback matrices
type = 'y=sin(t)';
model = 'P';
func_opt = @(type, kp_v, kp_th) P_model_V2(type, kp_v, kp_th);

% Turn off plotting
plotting = 0;

% Try 100 trials
ntrials = 100;
err_total = zeros(1,ntrials);
load(['best_',model,'model_',type])
for n = 1:ntrials
    fprintf('Trial:\n')
    disp(n)
    kp_v = [3*rand, 3*rand, .5*rand];
    kp_th = [.3*rand, .3*rand, .5*rand];
    err_total(n) = func_opt(type, kp_v, kp_th);
    
    % Check if error is now better
    if err_total(n)<err_best
        err_best = err_total;
        kp_v_best = kp_v;
        kp_th_best = kp_th;
        % Save best error and inputs
        fprintf('Saving new best!\n')
        save(['best_',model,'model_',type], 'err_best', 'kp_v_best', 'kp_th_best')
    end
end

