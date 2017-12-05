% Time steps and spans
tspan = [0,10];
DT = .01;
t = linspace(tspan(1), tspan(2), DT);

% Set up state matrices
Ah = @(k) [1, 0, -DT*cos(k*DT);...
           0, 1, -DT*sin(k*DT);...
           0, 0, 1];
Bh = @(k) [-DT*sin(k*DT), 0;...
            DT*cos(k*DT), 0;...
            0, DT];
        
% Create Ricatti Difference Matrices


% Loop through discrete time steps
for k = 1:length(t)
    
    
    
    
    
    
    
end