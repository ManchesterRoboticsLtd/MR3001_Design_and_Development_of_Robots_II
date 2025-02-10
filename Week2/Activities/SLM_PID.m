%% Dynamical System (Single Link Manipulator system)

%% Initialise simulation
clc
clear
close all

%% System parameters
m = 3.0; 
l = 0.40;
g = 9.8;
a = l/2;
J = (4/3)*m*a^2;
k_f = 0.0;

%% Controller Parameters
Kp = 12.0;
Ki = 20.0;
Kd = 2.4;

%% Set Point
set_point = 0.3;
%% Simulation Parameters
% Initial Conditions
theta_dot_l_0 = 0;
theta_l_0= 0.0;

% Sampling time
dt=0.01;

% Final time
tf=10;

%% Euler Approximation configuration
% Vector initialisation
t=0:dt:tf;

x1 = zeros(length(t),1);
x2 = zeros(length(t),1);
e = zeros(length(t),1);
ed = zeros(length(t),1);
e_int = zeros(length(t),1);
tau = zeros(length(t),1);
sp = zeros(length(t),1);

x1(1)=theta_l_0 ;
x2(1)=theta_dot_l_0;

%% Euler Approximation of the Solution
for k=1:length(t)-1
       
     sp(k) = set_point;
     e(k) = sp(k) - x1(k);
     if k == 1
         ed(k) = (e(k)- 0)/dt;
     else
        ed(k) = (e(k)-e(k-1))/dt;
     end
     tau(k) = Kp * e(k) + Ki * e_int(k) - Kd * x2(k);

     x1(k+1)=x1(k)+dt*(x2(k));
     x2(k+1)=x2(k)+dt*(-m*g*a*cos(x1(k)) - k_f * x2(k) + tau(k))*(1/J);

     e_int(k+1)=e_int(k)+dt*e(k);
     

end

%% Plotting

% figure(1);
% grid on
% hold on
% axis([-2 2 -2 2])

% % Plot Solution
% plot3(x1,x2,'LineWidth',3,'color','r')
% 
% % Plot Vector Field of the Pendulum (uncomment)
% %q=quiver3(X1,X2,X3,x1dot_n,x2dot_n,x3dot_n,0.5);
% q=quiver3(X1,X2,X3,x1dot_n,x2dot_n,x3dot_n,0.5);
% q.LineWidth=1.5;


% xlabel ('$x_1$','interpreter','latex','FontSize',22)
% ylabel ('$x_2$','interpreter','latex','FontSize',22)
% zlabel ('$x_3$','interpreter','latex','FontSize',22)
% title ('SLM System','FontSize',20)

figure(2)
plot(t,x1,'LineWidth',3,'color','b')
hold
plot(t,sp,'LineWidth',3,'color','r')
plot(t,tau,'LineWidth',3,'color','g')
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$x_1, x_2$','interpreter','latex','FontSize',22)
title ('SLM System','FontSize',20)

 figure(3)
 plot3(t,x1,x2,'LineWidth',3,'color','b')
 xlabel ('$t$','interpreter','latex','FontSize',22)
 ylabel ('$x_1$','interpreter','latex','FontSize',22)
 zlabel ('$x_2$','interpreter','latex','FontSize',22)
 title ('SLM System','FontSize',20)