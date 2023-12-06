% camTest:  A test jig for evaluating the estimate3dFeatureLocation function.

clear;clc;

sensorParamsScript;
P.sensorParams = sensorParams;

% Set true location of 3D point
rXI = [0;0;0.5];

% Set state of quad at time first image is taken
S.statek.rI = [1;0;1];
S.statek.RBI = euler2dcm([0;0;pi]);

% Simulate first image and populate measurement arrays
[rx] = hdCameraSimulator(rXI,S,P);
M.rxArray{1} = rx;
M.RCIArray{1} = P.sensorParams.RCB * S.statek.RBI;
M.rcArray{1} = S.statek.rI + S.statek.RBI'*P.sensorParams.rc;

% Set state of quad at time second image is taken
S.statek.rI = [0.7071;0.7071;1];
S.statek.RBI = euler2dcm([0;0;5*pi/4]);

% Simulate second image and populate measurement arrays
[rx] = hdCameraSimulator(rXI,S,P);
M.rxArray{2} = rx;
M.RCIArray{2} = P.sensorParams.RCB * S.statek.RBI;
M.rcArray{2} = S.statek.rI + S.statek.RBI'*P.sensorParams.rc;

% Estimate 3D feature location
[rXIHat,Px] = estimate3dFeatureLocation(M,P);

% Compare rXI and rXIHat
disp('rXI compared with rXIHat (should be within a few cm): ');
[rXI  rXIHat]

% Examine diagonal elements of Px
disp(['Sqrt of diagonal elements of Px (errors in rXIHat should not be much ' ...
      'bigger than these numbers): ']);
sqrt(diag(Px))
