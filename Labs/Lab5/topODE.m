clear; clc; close all;

% Initial value of vector X
X0 = [0;1;1];

% Vector of simulation segment start times
tVecIn = [0:1:10]';
dtIn = tVecIn(2) - tVecIn(1);
N = length(tVecIn);

% Set parameter values.  In this example, we allow the D parameter to
% change with time.  
parameters.A = -1;
parameters.B = 2;
parameters.C = [0.2 -1.5;0 -2];
parameters.D = abs(mean(X0));

% Oversampling causes the ODE solver to produce output at a finer time
% resolution than dtIn. oversampFact is the oversampling factor.
oversampFact = 10;
dtOut = dtIn/oversampFact;

% Create empty storage vectors
tVecOut = [];
XMat = [];

% Set initial state 
Xk = X0;

% In this example, we run ode45 for N-1 segments of dtIn seconds each.  Note
% that the value of parameters.D gets updated at the end of each iteration.
% This is just as an example of something that changes from one iteration to
% the next.
for k = 1:N-1
  
    % Build the time vector for kth segment.  We oversample by a factor
    % oversampFact relative to the coarse timing of each segment because we
    % may be interested in dynamical behavior that is short compared to the
    % segment length.
    tspan = [tVecIn(k):dtOut:tVecIn(k+1)]';
    
    % Run ODE solver for segment
    [tVeck,XMatk] = ode45(@(t,X)dxdtODE(t,X,parameters), tspan, Xk);
    
    % Add the data from the kth segment to your storage vector
    tVecOut = [tVecOut; tVeck(1:end-1)];
    XMat = [XMat; XMatk(1:end-1,:)];
    
    % Prepare for the next iteration
    Xk = XMatk(end,:)';
    
    % An example of an input parameter to the ODE function that changes
    % from call to call
    parameters.D = abs(mean(Xk));
end
% Store the final state of the final segment
XMat = [XMat; XMatk(end,:)];
tVecOut = [tVecOut; tVeck(end,:)];

% Plot: Note the sharp transitions in X3 that occur when the parameter D
% changes value.
figure;
subplot(3,1,1)
plot(tVecOut,XMat(:,1));
title('State time history of example dynamical system');
ylabel('X_1')
grid on;
subplot(3,1,2)
plot(tVecOut,XMat(:,2));
ylabel('X_2')
grid on;
subplot(3,1,3)
plot(tVecOut,XMat(:,3));
ylabel('X_3')
grid on;
xlabel('Time (s)');

