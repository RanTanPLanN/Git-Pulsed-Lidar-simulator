%% Pulsed LIDAR simulator for MSc thesis
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PURPOSE: Write a stand-alone pulsed LIDAR simulator tool to use in the
% MSc thesis. The tool must be as generic as possible.
%
% The user inputs should be the LIDAR position, the inclination angle of
% the LIDAR (theta), the azimuthial angle (phi) over which the LIDAR will
% scan the exact location of interest, the probe length, and the range
% spacing (gap between the range gates).
%
% The output(s) should be the reconstructed velocity at that specific point
%(at least for the moment).
%
% NOTE: The scripts assumes ground based LIDARs therefore all the
% geometrical calculations are performed based on that.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 2nd, 2017
% Last edited: February 13, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% User input section %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%% Operational mode selection

% operational mode: 
% Select 'p' for PPI or 's' for staring mode.
operationMode = 's';

%% Selection of weighting function shape

% choose type of weighting function:
% 't' for triangular-shaped function
% 'g' for gaussian-shaped function
weightingFuncType = 'g';

%% Number of LIDARs

% choose number of LIDARs that participate in the campaign (1,2 or 3)
nLidars = 2;

%% Dummies - These variable are just for testing

% Dimensions of dummy domain. They will be erased once the real LES grid
% will be implemented
xDimension = 190;
yDimension = 300;
zDimension = 140;

% Each dimension consists of 2 points per unit length
pointsPerDimensionLength = 2;
xVector = linspace(-xDimension/2,xDimension/2,xDimension*pointsPerDimensionLength);
yVector = linspace(0,yDimension,yDimension*pointsPerDimensionLength);
zVector = linspace(0,zDimension,zDimension*pointsPerDimensionLength);

% Create dummy domain to try out the interpolation
[Domain.x, Domain.y, Domain.z] = meshgrid(xVector, yVector, zVector);

% mean wind speed (m/s)
meanWindSpeed = 8;

% Create dummy laminar velocity field
% uComp = meanWindSpeed*ones(size(Domain.x)) - 1.5*meanWindSpeed*rand(size(Domain.x));;
% uComp = meanWindSpeed*rand(size(Domain.x));
uComp = meanWindSpeed*ones(size(Domain.x));
% uComp(:,1:length(xVector)/2,:) = 1;
vComp = zeros(size(Domain.x));
wComp = zeros(size(Domain.x));


%% Position of 1st LIDAR

% Give the lidar position (we assume that this point is the origin of the
% beam). The values should be in METRES. Put the lidar in the middle of the
% x-dimension
Lidar(1).x = 0;%xDimension/2;
Lidar(1).y = 0;
Lidar(1).z = 0;

%% Information about the probe of the 1st LIDAR

% Gap between the Lidar and the first range gate. Number in METRES
probe(1).FirstGap = 40;

% probe length in METRES
probe(1).Length = 10;

% points per unit length for the probe
probe(1).PointsPerLength = 5;

%% Information about additional LIDARs 

if nLidars > 1
    
    % position of second LIDAR in cartesian coordinates
    Lidar(2).x = Lidar(1).x + xDimension/2;
    Lidar(2).y = 100;
    Lidar(2).z = 0; 

    % FirstGap, PointsPerLength and NRgates will remain the same.
    probe(2).FirstGap = probe(1).FirstGap;
    probe(2).PointsPerLength = probe(1).PointsPerLength;

    % change the probe length if needed 
    probe(2).Length = 15;
    
%     % RangeGateGap is again twice the Length
%     probe(2).RangeGateGap = 2*probe(2).Length;
    
    % check if there is a 3rd LIDAR and assign the ncessary values
    if nLidars == 3
        % position of 3rd LIDAR in cartesian coordinates
        Lidar(3).x = Lidar(1).x - xDimension/2;
        Lidar(3).y = 180;
        Lidar(3).z = 0; 

        % FirstGap, PointsPerLength and NRgates will remain the same.
        probe(3).FirstGap = probe(1).FirstGap;
        probe(3).PointsPerLength = probe(1).PointsPerLength;

        % change the probe length if needed 
        probe(3).Length = 12;

%         % RangeGateGap is again twice the Length
%         probe(3).RangeGateGap = 2*probe(3).Length;  
    end
end

%% 
if operationMode == 'p';
    
    % Distance between range gates (should be at least equal to the
    % probeLength). Now I put it twice the probeLength. It is subject to change
    % in the future.
    probe(1).RangeGateGap = 2*probe(1).Length;
    
    % Radial distance to the point where we want to measure. The distance
    % should be larger that FirstGap + Length/2
    probe(1).RadialDist2MeasurePoint = 57;

    % Inclination angle theta. The value must be in DEGREES. This angle also
    % corresponds to the point we want to measure.
    % WARNING: If 2 or less LIDARs are used for the measurement, the
    % inclination angle should be as small as possible to ensure small
    % contribution of the w-component in the reconstructed velocity.
    thetaLidar = nan(1,nLidars);
    thetaLidar(1) = 30;

    % convert thetaLidar to radians
    thetaLidar(1) = deg2rad(thetaLidar(1));

    % Here it is assumed that the LIDAR is facing directly at the point we want
    % to measure, therefore the azimuthal angle of the point of interest is 0.
    phiMeasurePoint = 0;

    % azimuthial angle phi. The value must be in DEGREES. The lidar will scan
    % from -phi to +phi.
    phiLidar = 25;
    phiStep = 5;

    % number of range gates we want to measure.
    % NOTE: The code will return 3 range gates per Lidar UNLESS the
    % measurement point is too far from the Lidar location. If this is true
    % and more range gates fit before the one that actually contain the
    % focal point, then more range gates will be returned. If that is the
    % case then the focal point is always included in the last range gate.
    probe(1).NRangeGates = 3;
    
    % include the number of range gates in the probe(2) structure as well.
    if (nLidars > 1)
        probe(nLidars).NRangeGates = probe(1).NRangeGates;
        probe(nLidars-1).NRangeGates = probe(1).NRangeGates;
        
        % RangeGateGap is again twice the Length
        probe(nLidars).RangeGateGap = 2*probe(nLidars).Length;
        probe(nLidars-1).RangeGateGap = 2*probe(nLidars-1).Length;

    end;
    
    % call 'PPImode' script
    PPImode
else
    % matrix with the input measurement points. The matrix is [n x 3]. 
    % Each row represents one point and each column represents the x,y,z 
    % coordinate of that point.
    CartInputPoints = [-30 -10 -80 30;200 80 20 80;70 70 10 70]';
    
    % call 'StaringMode' script
    StaringMode
end 

% add 3rd Lidar here
% if nLidars > 2
% end 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% End of User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% DONT CHANGE ANYTHING BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% call printLidarInfo script to display all the user choices
printLidarInfo


% domain2.x = xVector; domain2.y = yVector; domain2.z = zVector;
% 
% % preallocate cell array
% domain{length(xVector),length(yVector),length(zVector)} = [];
% for xx = 1:length(xVector)
%     for yy = 1:length(yVector)
%         for zz = 1:length(zVector)
%             domain{xx,yy,zz} = {[xVector(xx),yVector(yy),zVector(zz)]};
%         end
%     end
% end
% clear xx yy zz

% Example: how to access the y-coordinate of the grid point (10,42,14):
% domain{10,42,14}{1}(2)
%
% Instead of the for loops you can also do domain = {xVector,yVector,zVector};
% but then you access the same y-coordinate: 
% domain2{2}(42)
% Create u,v and w components of the wind. For each meter of the x,y,z
% dimensions, there are 2 points (so 1 point every 0.5 metres).


