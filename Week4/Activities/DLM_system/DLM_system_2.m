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
t=t0:dt:tf;

% Operating Point(Linearisation)
x_1_op = pi/2;
x_2_op = pi;
x_dot_1_op = 0;
x_dot_2_op = 0;
u_1_op = 0;
u_2_op = 0;



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


%% Linearisation
%Linearising the system using the jacobian
A_sym = simplify(jacobian(q_ddot,[q_1,q_2,q_dot_1,q_dot_2]));
B_sym = simplify(jacobian(q_ddot,[tau_1, tau_2]));

% Evaluating linearised model

% Evaluating at the operating point
A_sub = simplify(subs(A_sym, {q_1, q_2, q_dot_1, q_dot_2},{x_1_op, x_2_op, x_dot_1_op, x_dot_2_op}));
B_sub = simplify(subs(B_sym, {q_1, q_2, q_dot_1, q_dot_2, tau_1, tau_2}, {x_1_op, x_2_op, x_dot_1_op, x_dot_2_op, u_1_op, u_2_op}));

%Linear Model SS Representation
A = matlabFunction(A_sub);
B = matlabFunction(B_sub);


% Euler Approximation of the Solution (Linear)
for k=1:length(t)-1
    
    Delta_q_1 = Delta_q(1,k);
    Delta_q_2 = Delta_q(2,k);
    Delta_q_dot_1 = Delta_q(3,k);
    Delta_q_dot_2 = Delta_q(4,k);

    Delta_q(:,k+1) = Delta_q(:,k) + dt*(A()*Delta_q(:,k)+B()*Delta_u(:,k));

    q_state(:,k+1) = Delta_q(:,k+1) + [x_1_op, x_2_op, x_dot_1_op, x_dot_2_op]';

end




%% NON-LINEAR SYSTEM
% Euler Approximation configuration
% Vector initialisation
X = zeros(4,length(t));
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
    
    q_1 = X(1,k);
    q_2 = X(2,k);
    q_dot_1 = X(3,k);
    q_dot_2 = X(4,k);


    X(:,k+1) = X(:,k) + dt*[X([3,4],k);((M(q_2))^-1)*(U(:,k) - C(q_2,q_dot_1,q_dot_2)*X([3,4],k) - G(q_1,q_2))];
end

%% Plotting
fig = figure('Name','DLM Nonlinear Simulation');
grid on;
hold on;
axis([0, 2, -5, 10])

plot(t,q_state(1,:),'LineWidth',2,'color','r')
plot(t,q_state(2,:),'LineWidth',2,'color','b')

plot(t,X(1,:),'LineWidth',2,'color','g')
plot(t,X(2,:),'LineWidth',2,'color','y')