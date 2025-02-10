%% Dynamical System (Single Link Manipulator system)

%% Initialise simulation
clc
clear
close all

%% System parameters
L = 0.03;
R = 6.0;
G = 37.0;
K_b = 0.04;
K_m = K_b;
J_m = 0.0008;
m = 3.0; 
l = 0.36;
g = 9.8;
a = l/2;
J = (4/3)*m*a^2;
 %% Voltage
 V = 0.0;

%% Simulation Parameters
% Initial Conditions
i_a_0=0;
theta_dot_l_0=0;
theta_l_0=0;

% Sampling time
dt=0.001;

% Final time
tf=20;

%% Build meshgrid for the Vector Field
res=0.3;
[X1,X2,X3] = meshgrid(-5:res:5,-5:res:5, -5:res:5);

%% Define the system
x1dot=(X2);
x2dot=(G*K_m*X3 - m*g*a*cos(X1))*(1/(G^2*J_m+J+m*a^2));
x3dot=V/L - K_b*G/L*X2 - R/L*X3;

%% Normalise Vectors in the vector field
x1dot_n=(x1dot./(sqrt(x1dot.^2+x2dot.^2+x3dot.^2)));
x2dot_n=(x2dot./(sqrt(x1dot.^2+x2dot.^2+x3dot.^2)));
x3dot_n=(x3dot./(sqrt(x1dot.^2+x2dot.^2+x3dot.^2)));

%% Euler Approximation configuration

% Vector initialisation
t=0:dt:tf;
x1= zeros(length(t),1);
x2= zeros(length(t),1);
x3= zeros(length(t),1);
x1(1)=theta_l_0 ;
x2(1)=theta_dot_l_0;
x3(1)=i_a_0;

%% Euler Approximation of the Solution
for k=1:length(t)-1
     x1(k+1)=x1(k)+dt*(x2(k));
     x2(k+1)=x2(k)+dt*((G*K_m*x3(k) - m*g*a*cos(x1(k)))*(1/(G^2*J_m+J+m*a^2)));
     x3(k+1)=x3(k)+dt*(V/L - K_b*G/L*x2(k) - R/L*x3(k));
end

%% Plotting

figure(1);
grid on
hold on
axis([-2 2 -2 2])

% Plot Solution
plot3(x1,x2,x3,'LineWidth',3,'color','r')

% Plot Vector Field of the Pendulum (uncomment)
%q=quiver3(X1,X2,X3,x1dot_n,x2dot_n,x3dot_n,0.5);
q=quiver3(X1,X2,X3,x1dot_n,x2dot_n,x3dot_n,0.5);
q.LineWidth=1.5;


xlabel ('$x_1$','interpreter','latex','FontSize',22)
ylabel ('$x_2$','interpreter','latex','FontSize',22)
zlabel ('$x_3$','interpreter','latex','FontSize',22)
title ('SLM System','FontSize',20)

figure(2)
plot(t,x1,'LineWidth',3,'color','b')
hold
plot(t,x2,'LineWidth',3,'color','r')
plot(t,x3,'LineWidth',3,'color','g')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$x_1, x_2, x_3$','interpreter','latex','FontSize',22)
title ('SLM System','FontSize',20)

figure(3)
plot3(x1,x2,x3,'LineWidth',3,'color','b')
xlabel ('$x_1$','interpreter','latex','FontSize',22)
ylabel ('$x_2$','interpreter','latex','FontSize',22)
zlabel ('$x_3$','interpreter','latex','FontSize',22)
title ('SLM System','FontSize',20)