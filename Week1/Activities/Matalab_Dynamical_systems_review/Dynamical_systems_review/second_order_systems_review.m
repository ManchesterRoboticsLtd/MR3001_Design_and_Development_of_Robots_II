%% Second Order Systems Review
%{

d^2y/dt^2 + (2*z*w_n)*dy/dt + (w_n^2)*y(t) = u

where:
w_n -> Natural frequency
z -> Damping Coefficient
u -> input

poles
s = -z * w_n (+/-) w_n * sqrt(1-z^2)

Time constant (tau)
tau = 1/(z*wn)

Damped frequency (w_d)
w_d = w_n * sqrt(1-z^2)

%}

%% Main Code

%{
This code will simulate 4 different second order systems (y1, y2, y3, y4) with different parameters for w_n and z 

%}

%Init
clc 
clear
close all

% Initialise Variables
syms y1(t) y2(t) y3(t) y4(t);

% Setup the parameters for each system
z1 = 0;
w_n1 = 2;

z2 = 0.05;
w_n2 = 2;

z3 = 0.5;
w_n3 = 2;

z4 = 1;
w_n4 =2;

% Setup Inputs for each system
u1 = dirac(t); % for step change to "heaviside(t)" 
u2 = dirac(t);
u3 = dirac(t);
u4 = dirac(t);

%Set the differetnial of each system
Dy1 = diff(y1,t);
D2y1 = diff(y1,t,2);

D2y2=diff(y2,t,2);
Dy2=diff(y2,t);
 
Dy3=diff(y3,t);
D2y3=diff(y3,t,2);

Dy4=diff(y4,t);
D2y4=diff(y4,t,2);
 
%Set the equations of each system 
y1 = simplify(dsolve(D2y1(t) + (2*z1*w_n1)*Dy1(t) + (w_n1^2)*y1(t) == u1, [y1(0)==0.0, Dy1(0)==0.0]))
y2 = simplify(dsolve(D2y2(t) + (2*z2*w_n2)*Dy2(t) + (w_n2^2)*y2(t) == u2, [y2(0)==0.0, Dy2(0)==0.0]))
y3 = simplify(dsolve(D2y3(t) + (2*z3*w_n3)*Dy3(t) + (w_n3^2)*y3(t) == u3, [y3(0)==0.0, Dy3(0)==0.0]))
y4 = simplify(dsolve(D2y4(t) + (2*z4*w_n4)*Dy4(t) + (w_n4^2)*y4(t) == u4, [y4(0)==0.0, Dy4(0)==0.0]))

%Plotting
fplot(y1, [0 15],'LineWidth',3,'color','b');
hold
fplot(y2, [0 15],'LineWidth',3,'color','r');
fplot(y3, [0 15],'LineWidth',3,'color','g');
fplot(y4, [0 15],'LineWidth',3,'color','c');

xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)
