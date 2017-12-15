function cost = LQR_LQG_model(varargin)
% ES 158 Final Project
% Jason Rosenberg and Spencer Hallyburton
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update in each version:

% V1 - Original code created by Jason
% V2 - Spencer added capability for general reference input of any function
% of time
% V3 - Spencer added capability for real-time reference input
% RENAME: LQR_LQG_model - merge LQR and LQG approaches into one main script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global remove_K
switch nargin
    case 0
        model = 'LQR';
        plotting = 1;
        noise_overwrite = 0;
    case 1
        model = varargin{1};
        plotting = 1;
        noise_overwrite = 0;
    case 2
        model = varargin{1};
        plotting = varargin{2};
        noise_overwrite = 0;
    case 3
        model = varargin{1};
        plotting = varargin{2};
        noise_overwrite = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global sigmaV sigmaW
time = 10; %total time of simulation
set(0,'defaultAxesFontSize',24)

if ~noise_overwrite
    % Determine the size of the noise variances
    switch model
        case 'LQG'
            sigmaV = 0.01;
            sigmaW = 0.1;
        case 'LQG_fierce'
            sigmaV = 0.025;
            sigmaW = 0.25;
        case 'LQR'
            sigmaV = 0;
            sigmaW = 0;
        otherwise
            error('Did not understand the model type')
    end
else
    sigmaV = varargin{3}(1);
    sigmaW = varargin{3}(2);
end

% Remove Kalman filter if LQR is true and noise is non-zero
if strcmp(model, 'LQR') && sigmaV~=0
    remove_K = 1;
else
    remove_K = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if plotting
    str = {'Circle','Real-Time Circle','User Choice','Pseudo-Random','y=sin(t)','Other'};
    prompt = 'Select A Reference Path';
    [s,ok] = listdlg('PromptString',prompt,...
                    'SelectionMode','single',...
                    'ListString',str);
    if ~ok; error('No Selection Made'); end
else
    str = {'Circle'};
    s = 1;
end

% Determine reference path from the gui
syms t y
switch str{s}
    case 'Circle'
        x_func = cos(t);
        y_func = sin(t);
        plt_type = 'static';
    case 'Real-Time Circle'
        x_func = cos(t);
        y_func = sin(t);
        plt_type = 'dynamic';
    case 'User Choice'
        [x_func, y_func, ~] = draw_path;
        plt_type = 'dynamic';
    case 'Pseudo-Random'
        rng(1)
        x_func = rand*sin(t) + rand*cos(t) + rand*t;
        y_func = rand*sin(t) + rand*cos(t) + rand*t;
        plt_type = 'static';
    case 'y=sin(t)'
        x_func = t;
        y_func = sin(t);
        plt_type = 'static';
    case 'Other'
        x_func = t;
        y_func = sin(x_func);
        plt_type = 'static';
end

% Run the plot function to plot the results
cost = plot_model(x_func, y_func, time, plt_type, plotting);

