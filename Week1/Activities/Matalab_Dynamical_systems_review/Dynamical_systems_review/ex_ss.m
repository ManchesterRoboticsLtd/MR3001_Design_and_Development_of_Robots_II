%% Dynamical System (mass-spring-damper system)

% dy/dt + k*y(t) = u(t)

%% Initialise simulation
clc
clear
close all

%% Simulation Parameters
% Initial Conditions
y_0=1;

% Sampling time
dt=0.001;

% Final time
tf=15;

%% Build meshgrid for the Vector Field of the Pendulum System
res=0.4;
[T,Y] = meshgrid(0:res:tf,-2:res:2);

%% Define the system and lyapunov functions
% Pendulum system with friction
ydot=-Y+dirac(T);
tdot=T;

%% Normalise Vectors in the vector field
ydot_n=(ydot./(sqrt(ydot.^2+tdot.^2)));
tdot_n=(tdot./(sqrt(ydot.^2+tdot.^2)));

%% Euler Approximation configuration
% Vector initialisation
t=0:dt:tf;
y= zeros(length(t),1);
y(1)=y_0;

%% Euler Approximation of the Solution
for k=1:length(t)-1
     y(k+1)=y(k)+dt*(-y(k));
     t(k+1)=t(k)+dt;
end

%% Plotting
figure(1);
grid on
hold on
axis([0 15 -2 2])
% Plot Solution
plot(t,y,'LineWidth',3,'color','r')
% Plot Vector Field
q=quiver(T,Y,tdot_n,ydot_n,0.5);
q.LineWidth=1;
%labels
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y$','interpreter','latex','FontSize',22)
title ('First Order System','FontSize',20)

