%% Dynamical System (mass-spring-damper system)

% d^2y/dt^2 + (b/m)*dy/dt + (k/m)*y(t) = (1/m)*F

%% Initialise simulation
clc
clear
close all

%% System parameters
b_damper=2.0;
k_spring=1.0;
mass=1.0;
force=1.0;

%% Simulation Parameters
% Initial Conditions
x1_0=0;
x2_0=0;

% Sampling time
dt=0.001;

% Final time
tf=15;

%% Build meshgrid for the Vector Field of the Pendulum System
res=0.3;
[X1,X2] = meshgrid(-2:res:2,-2:res:2);

%% Define the system and lyapunov functions
% Pendulum system with friction
x1dot=X2;
x2dot=-(b_damper/mass)*X2-(k_spring/mass)*X1+force;

%% Normalise Vectors in the vector field
x1dot_n=(x1dot./(sqrt(x1dot.^2+x2dot.^2)));
x2dot_n=(x2dot./(sqrt(x1dot.^2+x2dot.^2)));

%% Euler Approximation configuration

% Vector initialisation
t=0:dt:tf;
x1= zeros(length(t),1);
x2= zeros(length(t),1);
x1(1)=x1_0;
x2(1)=x2_0;

%% Euler Approximation of the Solution
for k=1:length(t)-1
     x1(k+1)=x1(k)+dt*(x2(k));
     x2(k+1)=x2(k)+dt*(-(b_damper/mass)*x2(k)-(k_spring/mass)*x1(k)+(1/mass)*force);
end

%% Plotting

figure(1);
grid on
hold on
axis([-2 2 -2 2])

% Plot Solution
plot(x1,x2,'LineWidth',3,'color','r')

% Plot Vector Field of the Pendulum (uncomment)
q=quiver(X1,X2,x1dot_n,x2dot_n,0.5);
q.LineWidth=1.5;


xlabel ('$x_1$','interpreter','latex','FontSize',22)
ylabel ('$x_2$','interpreter','latex','FontSize',22)
title ('Mass-Spring-Damper System','FontSize',20)

figure(2)
plot(t,x1,'LineWidth',3,'color','b')
hold
plot(t,x2,'LineWidth',3,'color','r')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$x_1, x_2t$','interpreter','latex','FontSize',22)
title ('Mass-Spring-Damper System','FontSize',20)

figure(3)
plot3(t,x1,x2,'LineWidth',3,'color','b')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$x_1$','interpreter','latex','FontSize',22)
zlabel ('$x_2$','interpreter','latex','FontSize',22)
title ('Mass-Spring-Damper System','FontSize',20)