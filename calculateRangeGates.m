function [scan,Points,LengthDiscr,r] = calculateRangeGates(RadialDist2MeasurePoint,probeLength, PointsPerLength,RangeGateGap,FirstGap,NRangeGates,phiVector,thetaLidar,operationMode)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% PURPOSE: Calculate all the Range Gate coordinates given some basic
% numbers about the probe (e.g. probe length, number of points per unit
% length etc). 
%
% When the LIDARs operate in PPI mode this function returns the full PPI
% scan of each LIDAR, thus in total it returns NRangeGates x length(phiVector)
% beams. When they operate in Staring mode it returns 1 beam per focus point
% because each LIDAR sends only one beam for each point.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs:
%
% 1. RadialDist2MeasurePoint: Radial distance from LIDAR to measuring point
% 2. probeLength: The length of each probe.
% 3. PointsPerLength: Points per unit length that the probe consists of.
% 4. RangeGateGap: Gap between two consecutive range gates.
% 5. FirstGap: Minimum distance between the LIDAR and the measuring point.
% 6. NRangeGates: Number of range gates.
% 7. phiVector: vector with the azimuthal angles the LIDAR is scanning. The
% angles are measured in MATLAB convention.
% 8. thetaLidar: elevation angle of the LIDAR.
%
% Outputs:
%
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
% Last edited: February 14, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number points that form the probe
Points = probeLength*PointsPerLength;

% probePoints should be an odd number in order to fit a proper weighting
% function
Points = (~mod(Points,2))*(Points + 1) +...
    mod(Points,2)*Points;

% Discretize the probeLength
LengthDiscr = linspace(0,probeLength,Points);

% radial distance vector
r = RadialDist2MeasurePoint - probeLength/2 + LengthDiscr;

% the following part makes sense only if the LIDARs are performing PPI
% scans in various distances and therefore emit more than one range gate.
% In case they operate in staring mode, there is no need for more than 1 RG
% per point. All that is needed is a RG that will include the measurement
% point.
if (operationMode == 'p')
    
    % Number of range gates that fit before the measurement point.
    CloseRG = floor((RadialDist2MeasurePoint - 0.5*probeLength - FirstGap)/...
        (RangeGateGap + probeLength));

    if (CloseRG < 0), CloseRG = 0; end;

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
end
% At this point, all the probe points of a complete scan of the LIDAR have
% been calculated in spherical coordinates.

% Convert the previously calculate probe points into cartesian coordinates.
% Cartesian coordinates are easier to manipulate (the LES grid is also
% going to be in cartesian) and are also need in order to interpolate the
% velocity on the grid points.

% preallocate scan structure
scan(length(phiVector)).CartX = [];
for ii = 1:length(phiVector)
    [scan(ii).CartX,scan(ii).CartY,scan(ii).CartZ] = ...
    sph2cart(phiVector(ii),thetaLidar,r);
end


end % end of function
