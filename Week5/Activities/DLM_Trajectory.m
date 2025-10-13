clc
clear
close all
%% Parameters
l1=1.0;
l2=1.0;


%% Spline
t=[0 1 2 3 4];
x=[0.1 1 1.99 1 0.1];
y=[0 1 0 -1 0];
T=[1 t(1) t(1)^2 t(1)^3 0 0 0 0 0 0 0 0 0 0 0 0    
   0 1 2*t(1) 3*t(1)^2 0 0 0 0 0 0 0 0 0 0 0 0      
   1 t(2) t(2)^2 t(2)^3 0 0 0 0 0 0 0 0 0 0 0 0      
   0 1 2*t(2) 3*t(2)^2 0 -1 -2*t(2) -3*t(2)^2 0 0 0 0 0 0 0 0   
   0 0 2 6*t(2) 0 0 -2 -6*t(2) 0 0 0 0 0 0 0 0   
   0 0 0 0 1 t(2) t(2)^2 t(2)^3 0 0 0 0 0 0 0 0
   0 0 0 0 1 t(3) t(3)^2 t(3)^3 0 0 0 0 0 0 0 0     
   0 0 0 0 0 1 2*t(3) 3*t(3)^2 0 -1 -2*t(3) -3*t(3)^2 0 0 0 0 
   0 0 0 0 0 0 2 6*t(3) 0 0 -2 -6*t(3) 0 0 0 0   
   0 0 0 0 0 0 0 0 1 t(3) t(3)^2 t(3)^3 0 0 0 0
   0 0 0 0 0 0 0 0 1 t(4) t(4)^2 t(4)^3 0 0 0 0   
   0 0 0 0 0 0 0 0 0 1 2*t(4) 3*t(4)^2 0 -1 -2*t(4) -3*t(4)^2
   0 0 0 0 0 0 0 0 0 0 2 6*t(4) 0 0 -2 -6*t(4)   
   0 0 0 0 0 0 0 0 0 0 0 0 1 t(4) t(4)^2 t(4)^3
   0 0 0 0 0 0 0 0 0 0 0 0 1 t(5) t(5)^2 t(5)^3   
   0 0 0 0 0 0 0 0 0 0 0 0 0 1 2*t(5) 3*t(5)^2];

A=[x(1) 0 x(2) 0 0 x(2) x(3) 0 0 x(3) x(4) 0 0 x(4) x(5) 0];
B=[y(1) 0 y(2) 0 0 y(2) y(3) 0 0 y(3) y(4) 0 0 y(4) y(5) 0];
ax=inv(T)*A';
ay=inv(T)*B';
t1 = t(1):(t(end)-t(1))/100:t(end);

%% Cartesian positions, velocities and accelerations
for i=1:length(t1)
if t1(i) < t(2);
k=1;
elseif t1(i) < t(3);
k=2;
elseif t1(i) < t(4);
k=3;
else
k=4;
end
xc(i) = ax((k-1)*4+1)*1 + ax((k-1)*4+2)*t1(i) + ax((k-1)*4+3)*t1(i)^2 + ax((k-1)*4+4)*t1(i)^3;
yc(i) = ay((k-1)*4+1)*1 + ay((k-1)*4+2)*t1(i) + ay((k-1)*4+3)*t1(i)^2 + ay((k-1)*4+4)*t1(i)^3;
Vx(i)=ax((k-1)*4+2) + 2*ax((k-1)*4+3)*t1(i) + 3*ax((k-1)*4+4)*t1(i)^2;
Vy(i)=ay((k-1)*4+2) + 2*ay((k-1)*4+3)*t1(i) + 3*ay((k-1)*4+4)*t1(i)^2;
Ax(i)= 2*ax((k-1)*4+3) + 6*ax((k-1)*4+4)*t1(i);
Ay(i)= 2*ay((k-1)*4+3) + 6*ay((k-1)*4+4)*t1(i);
end
%% Angular positions, velocities and accelerations
for h=1:length(t1)
    q2(h)= acos((xc(h)^2+yc(h)^2-l1^2-l2^2)/(2*l1*l2));
    q1(h)= atan2(yc(h),xc(h))-asin((l2*sin(q2(h)))/(sqrt(xc(h)^2+yc(h)^2)));

    J = [-l1*sin(q1(h))-l2*sin(q1(h)+q2(h)), -l2*sin(q1(h)+q2(h)); l1*cos(q1(h))+l2*cos(q1(h)+q2(h)), l2*cos(q1(h)+q2(h))];
    Dq(:,h) = inv(J)*[Vx(h);Vy(h)];
    D2q(:,h)=inv(J)*([Ax(h);Ay(h)]+[l1*cos(q1(h))*Dq(1,h)^2+l2*cos(q1(h)+q2(h))*(Dq(1,h)+Dq(2,h))^2;l1*sin(q1(h))*Dq(1,h)^2+l2*sin(q1(h)+q2(h))*(Dq(1,h)+Dq(2,h))^2]);

end


%% Plots

figure(1)
subplot(3,1,1); plot(t1, xc,'LineWidth',2);set(gca,'FontSize',12); xlabel('t','FontSize',12); ylabel('x spline','FontSize',12);title('X position','FontSize',12,'FontWeight','bold');
set(gcf,'PaperPositionMode','auto')
hold on
scatter (t,x,50,'r')
grid on
hold off
subplot(3,1,2); plot(t1, yc,'LineWidth',2);set(gca,'FontSize',12); xlabel('t','FontSize',12); ylabel('y spline','FontSize',12);title('Y position','FontSize',12,'FontWeight','bold');
hold on
scatter (t,y,50,'r')
grid on
hold off
subplot(3,1,3); plot(xc, yc,'LineWidth',2);set(gca,'FontSize',12); xlabel('x_1','FontSize',12); ylabel('x_2','FontSize',12);title('Y position','FontSize',12,'FontWeight','bold');
grid on

figure(2)
subplot(2,1,1); plot(t1, Vx,'LineWidth',2);set(gca,'FontSize',12);grid on; xlabel('t','FontSize',12); ylabel('Vx spline','FontSize',12);title('X Velocity','FontSize',12,'FontWeight','bold');
subplot(2,1,2); plot(t1, Vy,'LineWidth',2);set(gca,'FontSize',12);grid on; xlabel('t','FontSize',12); ylabel('Vy spline','FontSize',12);title('Y Velocity','FontSize',12,'FontWeight','bold');

figure(3)
subplot(2,1,1); plot(t1, Ax,'LineWidth',2);set(gca,'FontSize',12); grid on; xlabel('t','FontSize',12); ylabel('Ax spline','FontSize',12);title('Ax Accelration','FontSize',12,'FontWeight','bold');
subplot(2,1,2); plot(t1, Ay,'LineWidth',2);set(gca,'FontSize',12); grid on; xlabel('t','FontSize',12); ylabel('Ay spline','FontSize',12);title('Ay Acceleration','FontSize',12,'FontWeight','bold');

figure(4)
subplot(3,1,1);plot(t1,q1,'LineWidth',2); xlabel('t','FontSize',14); ylabel('Q1 [rad]','FontSize',14);title('Q1 joint','FontSize',14);
grid on
subplot(3,1,2);plot(t1,q2,'LineWidth',2); set(gca,'FontSize',14);xlabel('t','FontSize',14); ylabel('Q2 [rad]','FontSize',14);title('Q2 joint','FontSize',14);
grid on
subplot(3,1,3);plot(q1,q2,'LineWidth',2); set(gca,'FontSize',14);xlabel('Q1 [rad]','FontSize',14); ylabel('Q2 [rad]','FontSize',14);title('Q1 vs Q2 joint','FontSize',14);
grid on

figure(5)
subplot(2,1,1);plot(t1,Dq(1,:),'LineWidth',2); set(gca,'FontSize',14); xlabel('t','FontSize',14); ylabel('DQ1 [rad/s]','FontSize',14);title('Q1 joint Velocity','FontSize',14);
axis([0 4 -4 4])
grid on
subplot(2,1,2);plot(t1,Dq(2,:),'LineWidth',2);set(gca,'FontSize',14); xlabel('t','FontSize',14); ylabel('DQ2 [rad/s]','FontSize',14);title('Q2 joint Velocity','FontSize',14);
axis([0 4 -4 4])
grid on

figure(6)
subplot(2,1,1);plot(t1,D2q(1,:),'LineWidth',2); xlabel('t','FontSize',14); ylabel('D2Q1 [rad/s^2]','FontSize',14);title('Q1 joint Acceleration','FontSize',14);
axis([0 4 -100 100])
grid on
subplot(2,1,2);plot(t1,D2q(2,:),'LineWidth',2); xlabel('t','FontSize',14); ylabel('D2Q2 [rad/s^2]','FontSize',14);title('Q2 joint Acceleration','FontSize',14);
axis([0 4 -10 20])
grid on

