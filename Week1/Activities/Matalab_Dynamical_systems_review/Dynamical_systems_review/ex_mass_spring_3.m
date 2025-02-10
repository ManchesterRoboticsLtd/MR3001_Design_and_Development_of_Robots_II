%% Dynamical System (mass-spring-damper system)

% d^2y/dt^2 + (b/m)*dy/dt + (k/m)*y(t) = (1/m)*F

%% Initialise Sim
clc
clear all
close all

%% System Parameters
m = 1.0; k = 1.0; b = 2.0; f = 1;


gt = linspace(0,15,1000);
g = heaviside(gt);

tspan = [0 15];
[t,y] = ode45(@(t,y) mass_spring_system(t,y,m,k,b,f,g,gt), tspan,[0 0]);

figure(6)
plot(t,y)