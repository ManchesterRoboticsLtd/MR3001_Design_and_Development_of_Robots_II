%% Dynamical System (mass-spring-damper system)

% d^2y/dt^2 + (b/m)*dy/dt + (k/m)*y(t) = (1/m)*F

%% Initialise Sim
clc
clear all
close all

%% System Parameters
m = 1.0; k = 1.0; b = 2.0; f = 1;

%% Solution using symbolic toolbox 
syms y(t);
Dy = diff(y,t);
D2y = diff(y,t,2);
y = simplify(dsolve(D2y(t) + (b/m)*Dy(t) + (k/m)*y(t) == (f/m)*heaviside(t), [y(0)==0, Dy(0)==0]));
figure(1)
ezplot(y, [-1 15]);

%% Solution using symbolic toolbox and Laplace
syms U Y y2 s t2;
U = laplace(heaviside(t2));
Y = U*((1/m)/(s^2+(b/m)*s+(k/m)));
y2 = simplify(ilaplace(Y));
figure(2)
ezplot(y2, [0 15]);