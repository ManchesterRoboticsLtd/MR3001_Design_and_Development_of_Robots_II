clc
clear
%close all

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

% Simulation Parameters

% Initial Conditions
x_1_0 = pi/2-.003;
x_2_0 = pi-0.003;
x_dot_1_0 = 0;
x_dot_2_0 = 0;

u_1_0 = 0;
u_2_0 = 0;

% Initial/Final time (s)
t0=0.0;
tf=2;

% Sampling time
dt=0.00001;



% Operating Point(Linearisation)
x_1_op = pi/2;
x_2_op = pi;
x_dot_1_op = 0;
x_dot_2_op = 0;
u_1_op = 0;
u_2_op = 0;

%% NON-LINEAR SYSTEM
% Euler Approximation configuration
% Vector initialisation
t=t0:dt:tf;
X = zeros(4,length(t));
U = zeros(2,length(t));

% Set initial conditions
X(1,1) = x_1_0;
X(2,1) = x_2_0;
X(3,1) = x_dot_1_0;
X(4,1) = x_dot_2_0;

U(1,1) = u_1_0;
U(2,1) = u_2_0;


% Euler Approximation of the Solution (Nonlinear)
for k=1:length(t)-1
    
    q_1 = X(1,k);
    q_2 = X(2,k);
    q_dot_1 = X(3,k);
    q_dot_2 = X(4,k);

    M = [m_1*a^2+M_1*l_1^2+m_2*(l_1^2+d^2+2*l_1*d*cos(q_2))+M_2*(l_1^2+l_2^2+2*l_1*l_2*cos(q_2)),  m_2*d*(l_1*cos(q_2)+d)+M_2*l_2*(l_1*cos(q_2)+l_2);...
    m_2*d*(l_1*cos(q_2)+d)+M_2*l_2*(l_1*cos(q_2)+l_2),  m_2*d^2+M_2*l_2^2];

    C = [-2*l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_2,  -l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_2; ...
    l_1*sin(q_2)*(m_2*d+M_2*l_2)*q_dot_1,  0];

    G = g*[m_1*a*cos(q_1)+M_1*l_1*cos(q_1)+m_2*(l_1*cos(q_1)+d*cos(q_1+q_2))+M_2*(l_1*cos(q_1)+l_2*cos(q_1+q_2)); cos(q_1+q_2)*(m_2*d+M_2*l_2)];

    U = [0;0];

    X(:,k+1) = X(:,k) + dt*[X([3,4],k);(M^-1)*(U - C*X([3,4],k) - G)];
end

%% LINEAR SYSTEM
% Euler Approximation configuration
% Vector initialisation

Delta_q = zeros(4,length(t));
Delta_u = zeros(2,length(t));
q_state = zeros(4,length(t));

% Set initial conditions
Delta_q(1,1) = x_1_0-x_1_op;
Delta_q(2,1) = x_2_0-x_2_op;
Delta_q(3,1) = x_dot_1_0-x_dot_1_op;
Delta_q(4,1) = x_dot_2_0-x_dot_2_op;

Delta_u(1,1) = u_1_0-u_1_op;
Delta_u(2,1) = u_2_0-u_2_op;

q_state(1,1) = x_1_0;
q_state(2,1) = x_2_0;
q_state(3,1) = x_dot_1_0;
q_state(4,1) = x_dot_2_0;

A = [[0, 0, 1, 0]; ...
    [0, 0, 0, 1]; ...
    [(49*(25*sin(x_1_op + 2*x_2_op) - 73*sin(x_1_op)))/(5*(10*cos(2*x_2_op) - 25)), (17150*sin(x_1_op) - 70*x_dot_1_op^2*cos(x_2_op) - 70*x_dot_2_op^2*cos(x_2_op) - 700*u_1_op*sin(2*x_2_op) + 700*u_2_op*sin(2*x_2_op) - 1000*u_2_op*sin(x_2_op)^3 + 1000*x_dot_1_op^2*cos(x_2_op)^2 + 280*x_dot_1_op^2*cos(x_2_op)^3 + 280*x_dot_2_op^2*cos(x_2_op)^3 - 24500*cos(x_2_op)^2*sin(x_1_op) + 2750*u_2_op*sin(x_2_op) - 700*x_dot_1_op^2 - 140*x_dot_1_op*x_dot_2_op*cos(x_2_op) + 560*x_dot_1_op*x_dot_2_op*cos(x_2_op)^3 + 4116*cos(x_1_op)*cos(x_2_op)*sin(x_2_op))/(25*(4*cos(x_2_op)^2 - 7)^2), -(4*sin(x_2_op)*((7*x_dot_1_op)/25 + (7*x_dot_2_op)/25 + (2*x_dot_1_op*cos(x_2_op))/5))/(5*((4*cos(x_2_op)^2)/25 - 7/25)), (sin(x_2_op)*(28*x_dot_1_op + 28*x_dot_2_op))/(20*sin(x_2_op)^2 + 15)]; ...
    [(49*(sin(x_1_op + x_2_op) + (14*sin(x_1_op))/5)*((2*cos(x_2_op))/5 + 7/25))/(5*((4*cos(x_2_op)^2)/25 - 7/25)) - (49*sin(x_1_op + x_2_op)*((4*cos(x_2_op))/5 + 32/25))/(5*((4*cos(x_2_op)^2)/25 - 7/25)), (25*((2*cos(x_2_op))/5 + 7/25)*((2*cos(x_2_op)*x_dot_2_op^2)/5 + (4*x_dot_1_op*cos(x_2_op)*x_dot_2_op)/5 + (49*sin(x_1_op + x_2_op))/5))/(4*cos(x_2_op)^2 - 7) - (25*((49*sin(x_1_op + x_2_op))/5 - (2*x_dot_1_op^2*cos(x_2_op))/5)*((4*cos(x_2_op))/5 + 32/25))/(4*cos(x_2_op)^2 - 7) - (2*sin(x_2_op)*(10*sin(x_2_op)*x_dot_2_op^2 + 20*x_dot_1_op*sin(x_2_op)*x_dot_2_op + 25*u_1_op - 245*cos(x_1_op + x_2_op) - 686*cos(x_1_op)))/(20*cos(x_2_op)^2 - 35) - (4*sin(x_2_op)*(2*sin(x_2_op)*x_dot_1_op^2 - 5*u_2_op + 49*cos(x_1_op + x_2_op)))/(4*cos(x_2_op)^2 - 7) + (100*sin(2*x_2_op)*((2*cos(x_2_op))/5 + 7/25)*((2*sin(x_2_op)*x_dot_2_op^2)/5 + (4*x_dot_1_op*sin(x_2_op)*x_dot_2_op)/5 + u_1_op - (49*cos(x_1_op + x_2_op))/5 - (686*cos(x_1_op))/25))/(4*cos(x_2_op)^2 - 7)^2 + (100*sin(2*x_2_op)*((4*cos(x_2_op))/5 + 32/25)*((2*sin(x_2_op)*x_dot_1_op^2)/5 - u_2_op + (49*cos(x_1_op + x_2_op))/5))/(4*cos(x_2_op)^2 - 7)^2, (4*sin(x_2_op)*((32*x_dot_1_op)/25 + (7*x_dot_2_op)/25 + (4*x_dot_1_op*cos(x_2_op))/5 + (2*x_dot_2_op*cos(x_2_op))/5))/(5*((4*cos(x_2_op)^2)/25 - 7/25)), (sin(x_2_op)*(20*x_dot_1_op + 20*x_dot_2_op)*((2*cos(x_2_op))/5 + 7/25))/(4*cos(x_2_op)^2 - 7)]];
 
B = [[0,0];...
    [0, 0];...
    [-7/(4*cos(x_2_op)^2 - 7),   (10*cos(x_2_op) + 7)/(4*cos(x_2_op)^2 - 7)];...
    [(10*cos(x_2_op) + 7)/(4*cos(x_2_op)^2 - 7), -(20*cos(x_2_op) + 32)/(4*cos(x_2_op)^2 - 7)]];

% Euler Approximation of the Solution (Linear)
for k=1:length(t)-1
    
    Delta_q_1 = Delta_q(1,k);
    Delta_q_2 = Delta_q(2,k);
    Delta_q_dot_1 = Delta_q(3,k);
    Delta_q_dot_2 = Delta_q(4,k);

    Delta_q(:,k+1) = Delta_q(:,k) + dt*(A*Delta_q(:,k)+B*Delta_u(:,k));

    q_state(:,k+1) = Delta_q(:,k+1) + [x_1_op, x_2_op, x_dot_1_op, x_dot_2_op]';

end

%% Plotting
fig = figure('Name','DLM Nonlinear Simulation');
grid on;
hold on;
plot(t,X(1,:),'LineWidth',2,'color','g')
plot(t,X(2,:),'LineWidth',2,'color','y')

plot(t,q_state(1,:),'LineWidth',2,'color','r')
plot(t,q_state(2,:),'LineWidth',2,'color','b')



