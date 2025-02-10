%% Dynamical System (mass-spring-damper system)

% d^2y/dt^2 + (b/m)*dy/dt + (k/m)*y(t) = (1/m)*F

%%
clc
clear all
close all

%% System Parameters
m = 1.0; k = 1.0; b = 2.0; f = 1;

%% Solution using Control System Toolbox
g = tf([(1/m)],[1 (b/m) (k/m)]);
figure(3)
step(g);
hold
impulse(g);	% Step and impulse response

ltiview(g);		   	% Handy GUI for analyzing systems

