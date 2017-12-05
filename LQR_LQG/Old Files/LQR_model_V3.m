% ES 158 Final Project
% Jason Rosenberg and Spencer Hallyburton
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update in each version:

% V1 - Original code created by Jason
% V2 - Spencer added capability for general reference input of any function
% of time
% V3 - Spencer added capability for real-time reference input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global sigmaV sigmaW
sigmaV = 0.01;
sigmaW = 0.1;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
str = {'Circle','Real-Time Circle','User Choice','Pseudo-Random','Other'};
prompt = 'Select A Reference Path';
[s,ok] = listdlg('PromptString',prompt,...
                'SelectionMode','single',...
                'ListString',str);
if ~ok; error('No Selection Made'); end

% Determine reference path
syms t y
switch str{s}
    case 'Circle'
        x_func = cos(t);
        y_func = sin(t);
        type = 'static';
    case 'Real-Time Circle'
        x_func = cos(t);
        y_func = sin(t);
        type = 'dynamic';
    case 'User Choice'
        [x_func, y_func, path] = draw_path;
        type = 'dynamic';
    case 'Pseudo-Random'
        rng(1)
        x_func = rand*sin(t) + rand*cos(t) + rand*t;
        y_func = rand*sin(t) + rand*cos(t) + rand*t;
        type = 'static';
    case 'Other'
        x_func = t;
        y_func = sin(x_func);
        type = 'static';
end

% Run appropriate LQR function
switch type
    case 'static'
        static_LQG_V1(x_func, y_func)
    case 'dynamic'
        dynamic_LQR_V2(x_func, y_func)
end




