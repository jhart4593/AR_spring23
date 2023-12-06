function [dxdt] = dxdtODE(t,x,parameters)
% template : 
% dxdtODE  :  Example ordinary differential equation (ODE) function set up
%             for use with any of Matlab's ODE solvers (e.g., ode45).  
%
% Note that t is not used in this equation but must be included as an input
% for ode45 to function properly

% Initialize dx/dt
dxdt = zeros(3,1);

% Read data from the struct "parameters" which contains parameters
A = parameters.A;
B = parameters.B;
C = parameters.C;
D = parameters.D;

% Calculate dx/dt
dxdt(1) = A*x(1) + B*sin(x(2));
dxdt(2:3) = C*x(2:3)+[0;D]*cos(x(1));

end

