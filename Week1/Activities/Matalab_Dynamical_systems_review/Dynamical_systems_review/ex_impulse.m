%% Init simulation
clc 
clear
close all

%% Use symbolic toolbox to define a 1st order ODE

% dy/dt + Ay(t) = u(t)

% For this case two first order systems (y1 and y2) will be simulated
% one with an impulse input and the other with no input but with initial
% conditions

% Define variables 
syms y1(t) y2(t);

%Input Definition
u=0.0;
u2 = dirac(t-0.000001);

%First order system
Dy1=diff(y1,t);
Dy2=diff(y2,t);

f1 = Dy1(t) + (1)*y1(t) == u;
f2 = Dy2(t) + (1)*y2(t) == u2;

% Solve ODE's
y1 = simplify(dsolve(f1, [y1(0)==1]))
y2 = simplify(dsolve(f2,[y2(0)==0]))

% Plot
figure(1)
fplot(y1,[0,15],'LineWidth',3,'color','b')
hold
fplot(y2,[0,15],"--",'LineWidth',3,'color','r')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y_1(t), y_2(t)$','interpreter','latex','FontSize',22)
title ('First order systems','FontSize',20)


%% Use symbolic toolbox to define a 2nd Order ODE

% d^2y/dt^2 + B*dy/dt + A*y(t) = u(t)

% Define variables
syms y3(t) y4(t);

%Input Definition
u3 = 0;
u4 = dirac(t-0.0000001);

%Secoond order system
Dy3=diff(y3,t);
D2y3=diff(y3,t,2);

Dy4=diff(y4,t);
D2y4=diff(y4,t,2);

%Solve ODE's
y3 = simplify(dsolve(D2y3(t) + (2)*Dy3(t) + (4)*y3(t) == u3, [y3(0)==0.0, Dy3(0)==1.0]))
y4 = simplify(dsolve(D2y4(t) + (2)*Dy4(t) + (4)*y4(t) == u4, [y4(0)==0.0, Dy4(0)==0.0]))

%% Plot
figure(2)
fplot(y3,[0,15],'LineWidth',3,'color','b')
hold
fplot(y4,[0,15],"--",'LineWidth',3,'color','r')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)