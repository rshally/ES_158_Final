function [ x_func, y_func, path ] = draw_path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PURPOSE: Function to draw an arbitrary path and model this using an
% 8-term Fourier series expansion
%
% Inputs: None
%
% Outputs: x_func --> Fourier representation of the x-trajectory
%          y_func --> Fourier representation of the y-trajectory
%          path   --> The path object for placement on an axis
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Draw figure
h = figure;
axes
xlim([-3, 3])
ylim([-3, 3])
fprintf('INSTRUCTIONS:\n Draw a reference path for the robot using the mouse\n\n')
path = imfreehand;
coords = getPosition(path);
close(h)

%% Fit a function on the X data
syms t
x_fit = fit(linspace(0, 10, size(coords,1))', coords(:,1), 'fourier8');

% Extract coefficient names and store them in the workspace
cnames = coeffnames(x_fit);
cvalues = coeffvalues(x_fit);
for n = 1:length(cnames)
    eval([cnames{n},'=',num2str(cvalues(n))]);
end

% Convert cfit to a symbolic function
x_func = a0 + a1*cos(t*w) + b1*sin(t*w) + ...
         a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) +...
         a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +...
         a6*cos(6*t*w) + b6*sin(6*t*w) + a7*cos(7*t*w) + b7*sin(7*t*w) +...
         a8*cos(8*t*w) + b8*sin(8*t*w);
              
%% Fit a function on the Y data
y_fit = fit(linspace(0, 10, size(coords,1))', coords(:,2), 'fourier8');

% Extract coefficient names and store them in the workspace
cnames = coeffnames(y_fit);
cvalues = coeffvalues(y_fit);
for n = 1:length(cnames)
    eval([cnames{n},'=',num2str(cvalues(n))]);
end

% Convert cfit to a symbolic function
y_func = a0 + a1*cos(t*w) + b1*sin(t*w) + ...
         a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) +...
         a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +...
         a6*cos(6*t*w) + b6*sin(6*t*w) + a7*cos(7*t*w) + b7*sin(7*t*w) +...
         a8*cos(8*t*w) + b8*sin(8*t*w);

end

