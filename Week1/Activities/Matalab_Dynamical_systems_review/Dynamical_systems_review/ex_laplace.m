%% Init simulation
clc 
clear
close all
%% Use symbolic toolbox to define a 1st order ODE in Laplace

% dy/dt + Ay(t) = u(t), u(t)=dirac(t)
% G(s)=1/s+A

% Define variables 
syms s t;
 
%Define TF and invert laplace
U=laplace(heaviside(t))
Y = U*((1)/(s^2+3*s+2));
y = simplify(ilaplace(Y))

% Plot
figure(1)
fplot(y,[0,15],'LineWidth',3,'color','b')
hold
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('First order systems','FontSize',20)