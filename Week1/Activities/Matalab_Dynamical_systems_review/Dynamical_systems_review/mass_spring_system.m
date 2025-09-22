function dydt = mass_spring_system(t,y,m,k,b,f,g,gt)
g = interp1(gt,g,t); % Interpolate the data set (gt,g) at time t
dydt = [y(2); -(b/m)*y(2)-(k/m)*y(1)+(1/m)*g];
end