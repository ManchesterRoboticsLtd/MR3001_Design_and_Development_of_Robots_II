%% Init simulation
clc 
clear
close all

%% Use symbolic toolbox to define a 1st order ODE

% dy/dt + Ay(t) = u(t)

% Define variables
syms y(t);

%Input Definition
u=heaviside(t); %Impulse

%First order system
Dy=diff(y,t); %Define Derivatives
f1 = Dy(t) + (1)*y(t) == u; %Define Functions

%% Solve ODE
y = simplify(dsolve(f1, [y(0)==0.0]))

%% Plot
figure(1)
fplot(y,[0,15],'LineWidth',3,'color','b')
hold
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('First order system','FontSize',20)


%% Use symbolic toolbox to define a 2nd Order ODE

% d^2y/dt^2 + B*dy/dt + A*y(t) = u(t)

% Define variables
syms y2(t);
 
%Input definition
u2 = heaviside(t); %Step input
 
%Second order system

Dy2=diff(y2,t); %Derivatives definition
D2y2=diff(y2,t,2);

%% Solve ODE
y2 = simplify(dsolve(D2y2(t) + (2)*Dy2(t) + (4)*y2(t) == u2, [y2(0)==0.0, Dy2(0)==0.0]))

%% Plot
figure(2)
fplot(y2,[0,15],'LineWidth',3,'color','b')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y_2(t)$','interpreter','latex','FontSize',22)
title ('Second order system','FontSize',20)

