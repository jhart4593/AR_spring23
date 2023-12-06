function [rpGtilde,rbGtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rpGtilde --- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rbGtilde --- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%              is the known baseline distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

RIG = Recef2enu(P.sensorParams.r0G);
RPa = chol(RIG'*P.sensorParams.RpL*RIG);
RIB = S.statek.RBI'; 
rpI = S.statek.rI + RIB*P.sensorParams.raB(:,1);
rpG = RIG'*rpI;
rpGtilde = rpG + RPa'*randn(3,1);
rsI = S.statek.rI + RIB*P.sensorParams.raB(:,2);
rsG = RIG'*rsI;
rbG = rsG - rpG; 
rbGu = rbG/norm(rbG);
% Add an epsilon along the diagonal to ensure RbG is positive definite so we
% can apply Cholesky decomposition
epsilon = 1e-8;
RbG = norm(rbG)^2*(P.sensorParams.sigmab^2)*(eye(3)-rbGu*rbGu') + epsilon*eye(3);
wbG = chol(RbG)'*randn(3,1);
rbGtilde = rbG + wbG;
rbGtilde = norm(rbG)*rbGtilde/norm(rbGtilde);

