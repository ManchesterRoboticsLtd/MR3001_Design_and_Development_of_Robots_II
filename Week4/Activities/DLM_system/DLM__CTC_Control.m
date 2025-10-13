clc
clear
close all

% Model Parameters
m_1 = 3;
M_1 = 1.5;
m_2 = 3.0;
M_2 = 1.0;
a = 0.2;
b = 0.2;
d = 0.2;
e = 0.2;
l_1 = a+b;
l_2 = d+e;
g = 9.8;

% Controller Parameters
Kp1 = 100;
Kp2 = 100;

Kd1 = 20;
Kd2 = 20;

% Simulation Parameters
% Initial Conditions
x_1_0 = 0;
x_2_0 = 0;
x_dot_1_0 = 0;
x_dot_2_0 = 0;

u_1_0 = 0;
u_2_0 = 0;

% Initial/Final time (s)
t0=0.0;
tf=5;

% Sampling time
dt=0.00001;
t=t0:dt:tf;

%% NonLinear system Matrix Definition
syms q_1 q_2 q_dot_1 q_dot_2 q_ddot_1 q_ddot_2  tau_1 tau_2

q = [q_1; q_2];
q_dot = [q_dot_1; q_dot_2];

M = [m_1*a^2+M_1*l_1^2+m_2*(l_1^2+d^2+2*l_1*d*cos(q_2))+M_2*(l_1^2+l_2^2+2*l_1*l_2*cos(q_2)),  m_2*d*(l_1*cos(q_2)+d)+M_2*l_2*(l_1*cos(q_2)+l_2);...
    m_2*d*(l_1*cos(q_2)+d)+M_2*l_2*(l_1*cos(q_2)+l_2),  m_2*d^2+M_2*l_2^2];

C = [-2*l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_2,  -l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_2; ...
    l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_1,  0];

G = g*[m_1*a*cos(q_1)+M_1*l_1*cos(q_1)+m_2*(l_1*cos(q_1)+d*cos(q_1+q_2))+M_2*(l_1*cos(q_1)+l_2*cos(q_1+q_2)); cos(q_1+q_2)*(m_2*d+M_2*l_2)];

Tau = [tau_1; tau_2];

% Nonlinear model
q_ddot = simplify([q_dot; (M^-1)*(Tau - C*q_dot - G)]);



%% NON-LINEAR SYSTEM
% Euler Approximation configuration
% Vector initialisation
X = zeros(4,length(t));
r = zeros(6,length(t));
tau = zeros(2,length(t));
error_plot = zeros(2,length(t));
U = zeros(2,length(t));

% Set initial conditions
X(1,1) = x_1_0;
X(2,1) = x_2_0;
X(3,1) = x_dot_1_0;
X(4,1) = x_dot_2_0;

U(1,1) = u_1_0;
U(2,1) = u_2_0;

M = matlabFunction(M);  
G = matlabFunction(G);  
C = matlabFunction(C);  

% Euler Approximation of the Solution (Nonlinear)
for k=1:length(t)-1
    
    %Change of variables for simplicity in matrix substitution
    q_1 = X(1,k);
    q_2 = X(2,k);
    q_dot_1 = X(3,k);
    q_dot_2 = X(4,k);

    %Set Points
    %Link 1
    r1 = 1 + 0.3*sin(t(k));
    r1_dot = 0.3*cos(t(k));
    r1_dot_dot = -0.3*sin(t(k));

    %Link 2
    r2 = 0.5 + 0.7*sin(t(k));
    r2_dot = 0.7*cos(t(k));
    r2_dot_dot = -0.7*sin(t(k));

    r(:,k)=[r1;r1_dot;r1_dot_dot;r2;r2_dot;r2_dot_dot];

    error = [r1-q_1;r2-q_2];
    error_dot = [r1_dot-q_dot_1; r2_dot-q_dot_2];

    Kp_arr = [Kp1, 0; 0, Kp2];
    Kd_arr = [Kd1, 0; 0, Kd2];

    U = -Kp_arr*error - Kd_arr*error_dot;
    
    q_dot_dot_star = [r1_dot_dot;r2_dot_dot];

    Tau = M(q_2)*(q_dot_dot_star-U) + C(q_2,q_dot_1,q_dot_2)*X([3,4],k) + G(q_1,q_2); 
    

    X(:,k+1) = X(:,k) + dt*[X([3,4],k);((M(q_2))^-1)*(Tau - C(q_2,q_dot_1,q_dot_2)*X([3,4],k) - G(q_1,q_2))];
    
    error_plot(:,k)=error;
    tau(:,k)=Tau;

end

%% Plotting
fig = figure('Name','DLM Nonlinear Simulation');
grid on;
hold on;
axis([0, 10, -1, 2])

plot(t,r(1,:),'LineWidth',1)
plot(t,r(4,:),'LineWidth',1)
plot(t,X(1,:),'LineWidth',2,'color','g')
plot(t,X(2,:),'LineWidth',2,'color','y')

fig2 = figure('Name','Error');
grid on;
hold on;
axis([0, 10, -1, 2])

plot(t,error_plot(1,:),'LineWidth',1)
plot(t,error_plot(2,:),'LineWidth',1)

fig3 = figure('Name','Tau');
grid on;
hold on;
axis([0, 10, -1, 2])

plot(t,tau(1,:),'LineWidth',1)
plot(t,tau(2,:),'LineWidth',1)