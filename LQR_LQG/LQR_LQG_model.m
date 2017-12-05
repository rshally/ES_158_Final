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
global sigmaV sigmaW
model = 'LQG'; %change this to specify noise variances

% Determine the size of the noise variances
switch model
    case 'LQG'
        sigmaV = 0.01;
        sigmaW = 0.1;
    case 'LQG_fierce'
        sigmaV = 0.05;
        sigmaW = 0.5;
    case 'LQR'
        sigmaV = 0;
        sigmaW = 0;
    otherwise
        error('Did not understand the model type')
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
str = {'Circle','Real-Time Circle','User Choice','Pseudo-Random','Other'};
prompt = 'Select A Reference Path';
[s,ok] = listdlg('PromptString',prompt,...
                'SelectionMode','single',...
                'ListString',str);
if ~ok; error('No Selection Made'); end

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
        [x_func, y_func, path] = draw_path;
        plt_type = 'dynamic';
    case 'Pseudo-Random'
        rng(1)
        x_func = rand*sin(t) + rand*cos(t) + rand*t;
        y_func = rand*sin(t) + rand*cos(t) + rand*t;
        plt_type = 'static';
    case 'Other'
        x_func = t;
        y_func = sin(x_func);
        plt_type = 'static';
end

% Run the plot function to plot the results
plot_model(x_func, y_func, plt_type)


