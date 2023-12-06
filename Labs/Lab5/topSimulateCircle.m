% Top-level script for calling simulateQuadrotorDynamics
clear; clc;

% Total simulation time, in seconds
Tsim = 5;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Set parameters for a circular trajectory
quadParamsScript;
constantsScript;
% Radius of circle
rCircle = 1;
% Tangent velocity
v0 = 2*pi*rCircle/Tsim;
% Centripetal acceleration
ac = v0^2/rCircle;
% Thrust required to counteract gravity
fg = constants.g*quadParams.m;
% Thrust required for centripetal acceleration
fac = quadParams.m*ac;
% Total scalar thrust
ft = sqrt(fg^2 + fac^2);
% Angle between ENU Up and zB
thetaTilt = atan2(fac, fg);
% Yaw rate
psidot = 2*pi/Tsim;
% Time vector, in seconds 
N = floor(Tsim/delt);
S.tVec = [0:N-1]'*delt;
% Matrix of disturbance forces acting on the body, in N, expressed in I
S.distMat = zeros(N-1,3);
% Initial position in m
S.state0.r = [rCircle 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 thetaTilt pi]';
e = S.state0.e;
% From kinematic equation for Euler angles
M = [cos(e(2)) 0 -sin(e(2))*cos(e(1));
     0  1  sin(e(1));
     sin(e(2)) 0 cos(e(2))*cos(e(1))];
omegaB0 = M*[0;0;psidot];
% Note that omegaB is constant.  This allows one to obtain NB from Euler's
% equation:
NB = crossProductEquivalent(omegaB0)*quadParams.Jq*omegaB0;
% NB indicates that we only need a torque about the body Y axis.  One can
% solve for the exact rotor rates that produce NB and ft.  This requires
% forming and inverting a 4-by-4 matrix that we'll learn about in later
% lectures.  Alternatively, one can simply note that the front and back rotors
% must spin at slightly different rates, as follows:
epsw = 2.255e-4;
% Rotor speeds at each time, in rad/s
beta = sqrt(ft/fg);
S.omegaMat = beta*585.65*[(1-epsw)*ones(N-1,2), (1+epsw)*ones(N-1,2)];
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 v0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = omegaB0;
% Oversampling factor
S.oversampFact = 10;
% Quadrotor parameters and constants
S.quadParams = quadParams;
S.constants = constants;
P = simulateQuadrotorDynamics(S);

S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=2*[-1 1 -1 1 -1 1];
visualizeQuad(S2);

figure(1);clf;
plot(P.tVec,P.state.rMat(:,3)); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(2);clf;
plot(P.state.rMat(:,1), P.state.rMat(:,2)); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');