%% Second Order Systems Review
%{

d^2y/dt^2 + (2*z*w_n)*dy/dt + (w_n^2)*y(t) = a*Du + b*u

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
syms y1(t) y2(t) y3(t) y4(t) u1(t) u2(t) u3(t) u4(t) s;

% Setup the parameters for each system
z1 = 0;
w_n1 = 2;
a1 = 2;
b1 = 1;

z2 = 0.05;
w_n2 = 2;
a2 = 2;
b2 = 1;

z3 = 0.5;
w_n3 = 2;
a3 = 2;
b3 = 1;

z4 = 1;
w_n4 =2;
a4 = 2;
b4 = 1;

% Setup Inputs for each system
u1(t) = dirac(t); % for step change to "heaviside(t) or "dirac(t)" or "sin(t)", etc" 
u2(t) = dirac(t);
u3(t) = dirac(t);
u4(t) = dirac(t);

%Set the differetnial equations of each system
Du1 = diff(u1,t);
Dy1 = diff(y1,t);
D2y1 = diff(y1,t,2);
f1 = D2y1(t) + (2*z1*w_n1)*Dy1(t) + (w_n1^2)*y1(t);
i1 = a1*Du1(t) + b1*u1(t);

Du2 = diff(u2,t);
D2y2 = diff(y2,t,2);
Dy2 = diff(y2,t);
f2 = D2y2(t) + (2*z2*w_n2)*Dy2(t) + (w_n2^2)*y2(t);
i2 = a2*Du2(t) + b2*u2(t);
 
Du3 = diff(u3,t);
Dy3 = diff(y3,t);
D2y3 = diff(y3,t,2);
f3 = D2y3(t) + (2*z3*w_n3)*Dy3(t) + (w_n3^2)*y3(t)
i3 = a3*Du3(t) + b3*u3(t);

Du4 = diff(u4,t);
Dy4 = diff(y4,t);
D2y4 = diff(y4,t,2);
f4 = D2y4(t) + (2*z4*w_n4)*Dy4(t) + (w_n4^2)*y4(t);
i4 = a4*Du4(t) + b4*u4(t);
 
%Set the equations of each system 
y1 = simplify(dsolve(f1==u1, [y1(0)==0.0, Dy1(0)==0.0]))
y2 = simplify(dsolve(f2==u2, [y2(0)==0.0, Dy2(0)==0.0]))
y3 = simplify(dsolve(f3==u3, [y3(0)==0.0, Dy3(0)==0.0]))
y4 = simplify(dsolve(f4==u4, [y4(0)==0.0, Dy4(0)==0.0]))

%Poles and zeros Transform
[X,Y] = meshgrid(-3: 0.2: 3);
s = X + 1i*Y;

Y1  = abs(((a1.*s+b1) ./ ((s.^2) + (2*z1*w_n1).*s + w_n1^2)));
Y2  = abs(((a2.*s+b2) ./ ((s.^2) + (2*z2*w_n2).*s + w_n2^2)));
Y3  = abs(((a3.*s+b3) ./ ((s.^2) + (2*z3*w_n3).*s + w_n3^2)));
Y4  = abs(((a4.*s+b4) ./ ((s.^2) + (2*z4*w_n4).*s + w_n4^2)));


figure(1);
subplot(2,1,1);
surf(X,Y,Y1)
hold
contour(X,Y,Y1)
xlim([-3 3]);
ylim([-3 3]);
zlim([-1 5]);
pbaspect([1 1 1])
xlabel ('$real$','interpreter','latex','FontSize',22)
ylabel ('$imaginary$','interpreter','latex','FontSize',22)
zlabel ('$amplitude$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)
subplot(2,1,2); 
fplot(y1, [0 15],'LineWidth',3,'color','b');
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)

figure(2);
subplot(2,1,1);
surf(X,Y,Y2)
hold
contour(X,Y,Y1)
xlim([-3 3]);
ylim([-3 3]);
zlim([-1 5]);
pbaspect([1 1 1])
xlabel ('$real$','interpreter','latex','FontSize',22)
ylabel ('$imaginary$','interpreter','latex','FontSize',22)
zlabel ('$amplitude$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)
subplot(2,1,2); 
fplot(y2, [0 15],'LineWidth',3,'color','b');
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)

figure(3);
subplot(2,1,1);
surf(X,Y,Y3)
hold
contour(X,Y,Y1)
xlim([-3 3]);
ylim([-3 3]);
zlim([-1 5]);
pbaspect([1 1 1])
xlabel ('$real$','interpreter','latex','FontSize',22)
ylabel ('$imaginary$','interpreter','latex','FontSize',22)
zlabel ('$amplitude$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)
subplot(2,1,2); 
fplot(y3, [0 15],'LineWidth',3,'color','b');
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)

figure(4);
subplot(2,1,1);
surf(X,Y,Y4)
hold
contour(X,Y,Y1)
xlim([-3 3]);
ylim([-3 3]);
zlim([-1 5]);
pbaspect([1 1 1])
xlabel ('$real$','interpreter','latex','FontSize',22)
ylabel ('$imaginary$','interpreter','latex','FontSize',22)
zlabel ('$amplitude$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)
subplot(2,1,2); 
fplot(y4, [0 15],'LineWidth',3,'color','b');
xlabel ('$t$','interpreter','latex','FontSize',22)
ylabel ('$y(t)$','interpreter','latex','FontSize',22)
title ('Second order systems','FontSize',20)

