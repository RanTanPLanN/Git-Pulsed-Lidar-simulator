function [scan,Points,LengthDiscr,r] = calculateRangeGates(RadialDist2MeasurePoint,probeLength, PointsPerLength,RangeGateGap,FirstGap,NRangeGates,phiVector,thetaLidar)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PURPOSE: Calculate all the Range Gate coordinates given some basic
% numbers about the probe (e.g. probe length, number of points per unit
% length etc).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs:
%
%
%
%
% Outputs:
% 1. scan: Structure that contains the cartesian coordinates of the range
% gates over the scanned area.
% 2. Points: Scalar number equal to the number of points per range gate
% (number discretized points that a range gate consists of).
% 3. LengthDiscr: Vector from 0 to probeLength with length equal to Points 
% (see above).  
% 4. r: Vector with all the radial distances that the LIDAR is scanning
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 9, 2017
% Last edited: February 9, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number points that form the probe
Points = probeLength*PointsPerLength;

% probePoints should be an odd number in order to fit a proper triangular
% weighting function
Points = (~mod(Points,2))*(Points + 1) +...
    mod(Points,2)*Points;

% Discretize the probeLength
LengthDiscr = linspace(0,probeLength,Points);

% Number of range gates that fit before the measurement point
CloseRG = floor((RadialDist2MeasurePoint - 0.5*probeLength - FirstGap)/...
    (RangeGateGap + probeLength));
if (CloseRG < 0) CloseRG = 0; end;

% radial distance vector
r = RadialDist2MeasurePoint - probeLength/2 + LengthDiscr;

% previousProbe = probe.r-probe.RangeGateGap-probe.Length;
previousProbe = r;
for ii = 1:CloseRG
    previousProbe = previousProbe - RangeGateGap - probeLength;
    r = [previousProbe, r];
end

% Calculate the radial distances of all the points of all the Range Gates
for ii = 1:NRangeGates - CloseRG - 1
    % calculate next Range Gate
    nextRG = r(end) + RangeGateGap + LengthDiscr;
    r = [r, nextRG];
end

scan(length(phiVector)).CartX = [];
for ii = 1:length(phiVector)
    [scan(ii).CartX,scan(ii).CartY,scan(ii).CartZ] = ...
    sph2cart(phiVector(ii),thetaLidar,r);
end


end % end of function
