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
%
% Created: February 2nd, 2017
% Last edited: February 7, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% User input section %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

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

% mean wind speed (m/s)
meanWindSpeed = 8;
%%

% Give the lidar position (we assume that this point is the origin of the
% beam). The values should be in METRES. Put the lidar in the middle of the
% x-dimension
Lidar(1).x = 0;%xDimension/2;
Lidar(1).y = 0;
Lidar(1).z = 0;


% Radial distance to the point where we want to measure. The distance
% should be larger that FirstGap + Length/2
probe(1).RadialDist2MeasurePoint = 57;

% Inclination angle theta. The value must be in DEGREES. This angle also
% corresponds to the point we want to measure.
% WARNING: If 2 or less LIDARs are used for the measurement, the
% inclination angle should be as small as possible to ensure small
% contribution of the w-component in the reconstructed velocity.
thetaLidar = 30;

% convert thetaLidar to radians
thetaLidar = deg2rad(thetaLidar);

% Here it is assumed that the LIDAR is facing directly at the point we want
% to measure, therefore the azimuthal angle of the point of interest is 0.
phiMeasurePoint = 0;

% azimuthial angle phi. The value must be in DEGREES. The lidar will scan
% from -phi to +phi.
phiLidar = 25;
phiStep = 5;

% Gap between the Lidar and the first range gate. Number in METRES
probe(1).FirstGap = 40;

% probe length in METRES
probe(1).Length = 10;

% points per unit length for the probe
probe(1).PointsPerLength = 5;

% number of range gates we want to measure
probe(1).NRangeGates = 3;

% choose type of weighting function:
% 't' for triangular-shaped function
% 'g' for gaussian-shaped function
weightingFuncType = 'g';

% choose number of LIDARs that participate in the campaign (1,2 or 3)
nLidars = 2;

%%%%%%%%%%%%%%%%%%%% Information about second LIDAR %%%%%%%%%%%%%%%%%%%%%%%
if nLidars > 1
    
    % position of second LIDAR in cartesian coordinates
    Lidar(2).x = Lidar(1).x + xDimension/2;
    Lidar(2).y = 100;
    Lidar(2).z = 0; 

    % FirstGap, PointsPerLength and NRgates will remain the same.
    probe(2).FirstGap = probe(1).FirstGap;
    probe(2).PointsPerLength = probe(1).PointsPerLength;
    probe(2).NRangeGates = probe(1).NRangeGates;

    % change the probe length if needed 
    probe(2).Length = 15;

    % RangeGateGap is again twice the Length
    probe(2).RangeGateGap = 2*probe(2).Length;
end

% add 3rd Lidar here
% if nLidars > 2
% end 
    

% For now the LIDARs are going to be similar (FirstGap, probeLength,
% phiLidar, weightingFunction etc). They will have different NRangeGates,
% thetaLidar and initial position -origin of the beam.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% End of User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% DONT CHANGE ANYTHING BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (probe(1).RadialDist2MeasurePoint <= probe(1).FirstGap + probe(1).Length/2)
    fprintf('Measurement point to close to LIDAR location.\nExiting...\n')
    return
end

disp('-------- Pulsed LIDAR simulator----------')
disp('-----------------------------------------')
disp(' ')
fprintf('Number of LIDARs: %d\n\n', nLidars) 

% Print all user choices in the command window
disp('------Information about 1st LIDAR--------')
disp(' ')

fprintf('Lidar Position:\n\n')
fprintf('x = %f\n',Lidar(1).x)
fprintf('y = %f\n',Lidar(1).y)
fprintf('z = %f\n\n\n',Lidar(1).z)
fprintf('Radial distance from point of interest: %.1f metres\n'...
    ,probe(1).RadialDist2MeasurePoint)

fprintf('Inclination angle: %.1f degrees.\n'...
    ,rad2deg(thetaLidar(1)))
fprintf('Azimuthial range: %.1f to %.1f degrees\n',-phiLidar,phiLidar)
fprintf('Azimuthial step: %.1f degrees\n\n\n', phiStep)

fprintf('Closest range: %.1f metres\n',probe(1).FirstGap);
fprintf('Probe length: %.1f metres\n',probe(1).Length)
fprintf('Points per unit length of the probe: %d\n',probe(1).PointsPerLength)
fprintf('Number of Range Gates: %d \n\n',probe(1).NRangeGates)

Shape = (weightingFuncType == 't')*'triangular' +...
    (weightingFuncType == 'g')*' Gaussian ';
fprintf('Shape of the weighting function: %s\n\n', Shape)
clear Shape

if (nLidars > 1)
    disp('------Information about 2nd LIDAR-------')
    disp(' ')
    fprintf('Lidar Position:\n\n')
    fprintf('x = %f\n',Lidar(2).x)
    fprintf('y = %f\n',Lidar(2).y)
    fprintf('z = %f\n',Lidar(2).z)
end
disp('----------------------------------------')
% end of printing information



% Create dummy domain to try out the interpolation
% domain dimensions. 
[Domain.x, Domain.y, Domain.z] = meshgrid(xVector, yVector, zVector);


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

% Create dummy laminar velocity field
% uComp = meanWindSpeed*ones(size(Domain.x)) - 1.5*meanWindSpeed*rand(size(Domain.x));;
% uComp = meanWindSpeed*rand(size(Domain.x));
uComp = meanWindSpeed*ones(size(Domain.x));
% uComp(:,1:length(xVector)/2,:) = 1;
vComp = zeros(size(Domain.x));
wComp = zeros(size(Domain.x));

% Create vector of azimuthial angles and convert it to radians. I have to
% add 90 degrees because of the way MATLAB converts spherical to cartesian 
% coordinates. For more info see:
% https://www.mathworks.com/help/matlab/ref/sph2cart.html .
probe(1).phiVector = (-phiLidar:phiStep:phiLidar) + phiMeasurePoint;
probe(1).phiVector = deg2rad(-probe(1).phiVector + 90);

%% Probe 

% Distance between range gates (should be at least equal to the
% probeLength). Now I put it twice the probeLength. It is subject to change
% in the future.
probe(1).RangeGateGap = 2*probe(1).Length;

[Lidar(1).scan,probe(1).Points,probe(1).LengthDiscr,probe(1).r] = ...
    calculateRangeGates(probe(1).RadialDist2MeasurePoint,probe(1).Length...
    ,probe(1).PointsPerLength,probe(1).RangeGateGap,probe(1).FirstGap...
    ,probe(1).NRangeGates,probe(1).phiVector,thetaLidar(1));

%% Weighting function (this part might need to be a separate MATLAB function)

WeightFunc = WeightingFunction(probe(1).LengthDiscr,weightingFuncType);

% plot normalized weighting function
figure
plot(probe(1).LengthDiscr,WeightFunc)
hold on
plot([probe(1).LengthDiscr((end+1)/2) probe(1).LengthDiscr((end+1)/2)]...
    ,ylim,'k--')
hold off

%% Draw 3D - Careful
% Generates one figure per azimuthial angle phi -so in total the number of 
% figures will be length(phiVector)!!!
% Be careful if the phiLidar vector has very small step

% for ii = 1:length(phiVector)
%     figure
%     plot3(scan(ii).CartX, scan(ii).CartY, scan(ii).CartZ);
%     grid on
%     xlabel('x axis')
%     ylabel('y axis')
%     zlabel('z axis')
%     axis([-50 50 30 105 18 70 ])
% end

%% Plot the scanning of the LIDAR both from top and front view

figure
for ii = 1:length(probe(1).phiVector)
    clf
    hold on
    subplot(1,2,1)
    plot(Lidar(1).scan(ii).CartX, Lidar(1).scan(ii).CartY,'.')
    ylabel('y axis')
    xlabel('x axis')
    title('Top view')
    axis([-50 50 30 110])
    grid on
    
    subplot(1,2,2)
    plot(Lidar(1).scan(ii).CartX, Lidar(1).scan(ii).CartZ,'r.')
    ylabel('z axis')
    xlabel('x axis')
    axis([-50 50 18 70 ])
    title('Front view')
    grid on
    drawnow
end
hold off

%% Not needed - just a visual check
 
[measurePoint.x,measurePoint.y,measurePoint.z] = sph2cart(probe(1).phiVector((length(probe(1).phiVector)+1)/2),...
    thetaLidar(1),probe(1).RadialDist2MeasurePoint);
% figure
% plot3(Lidar(1).scan((length(probe(1).phiVector)+1)/2).CartX,...
%     Lidar(1).scan((length(probe(1).phiVector)+1)/2).CartY,...
%     Lidar(1).scan((length(probe(1).phiVector)+1)/2).CartZ,'.');
% hold on
% % plot3(scan((length(phiVector)+1)/2).CartX(end), scan(1).CartY(end), scan(1).CartZ(end),'*');
% plot3(measurePoint.x,measurePoint.y,measurePoint.z,'*')
% hold off
% grid on
% grid minor
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')
% axis([-50 50 30 105 18 70 ])

%% Calculate all distances between domain points and probe points

% distances(length(probe.r),length(phiVector)).dd = [];
% for pp = 1:length(phiVector)
%     for rr = 1:length(probe.r)
%     distances(rr,pp).dd = sqrt((Domain.x - scan(pp).CartX(rr)).^2 + (Domain.y - scan(pp).CartY(rr)).^2 + (Domain.z - scan(pp).CartZ(rr)).^2);
%     end
% end

% [minM, idx] = min(distances(1,1).dd(:));
% [n, m ,t] = ind2sub(size(distances(1,1).dd),idx);
% 
% % check
% minM
% distances(1,1).dd(n,m,t)

%% Locate the coordinates of the cube that contains the  last point of the probe
% This whole section is not necessary for the calculations it was just made
% for a quick visual check

condX = Lidar(1).scan(1).CartX(end)>=xVector;
differX = diff(condX);

% maybe this method is faster, i dont know yet.
Xlow = xVector(logical(abs(differX)));
Xhigh = xVector(logical([0 abs(differX)]));
CubeX = [Xlow Xhigh];

condY = Lidar(1).scan(1).CartY(end)>=yVector;
differY = diff(condY);
Ylow = yVector(logical(abs(differY)));
Yhigh = yVector(logical([0 abs(differY)]));
CubeY = [Ylow Yhigh];

condZ = Lidar(1).scan(1).CartZ(end)>=zVector;
differZ = diff(condZ);
Zlow = zVector(logical(abs(differZ)));
Zhigh = zVector(logical([0 abs(differZ)]));
CubeZ = [Zlow Zhigh];

[cube.x,cube.y,cube.z] = meshgrid(CubeX, CubeY, CubeZ);

% Plot for testing
% figure
% plot3(Lidar(1).scan(1).CartX(end), Lidar(1).scan(1).CartY(end), Lidar(1).scan(1).CartZ(end),'*');
% hold on
% % draw bottom xy-plane
% plot3([Xlow Xlow],[Ylow Yhigh],[Zlow Zlow],'r')
% plot3([Xlow Xhigh],[Yhigh Yhigh],[Zlow Zlow],'r')
% plot3([Xhigh Xhigh],[Yhigh Ylow],[Zlow Zlow],'r')
% plot3([Xhigh Xlow],[Ylow Ylow],[Zlow Zlow],'r')
% 
% % draw upper xy-plane
% plot3([Xlow Xlow],[Ylow Yhigh],[Zhigh Zhigh],'r')
% plot3([Xlow Xhigh],[Yhigh Yhigh],[Zhigh Zhigh],'r')
% plot3([Xhigh Xhigh],[Yhigh Ylow],[Zhigh Zhigh],'r')
% plot3([Xhigh Xlow],[Ylow Ylow],[Zhigh Zhigh],'r')
% 
% % connect the xy-planes
% plot3([Xlow Xlow],[Ylow Ylow],[Zlow Zhigh],'r')
% plot3([Xhigh Xhigh],[Ylow Ylow],[Zlow Zhigh],'r')
% plot3([Xlow Xlow],[Yhigh Yhigh],[Zlow Zhigh],'r')
% plot3([Xhigh Xhigh],[Yhigh Yhigh],[Zlow Zhigh],'r')
% 
% hold off
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')
% grid on
% grid minor

%% Interpolate the grid velocity values to obtain the velocity at the probe points

% interpolated velocity components and store them in interpVel structure.
% First pre-allocate the structure.
interpVel(nLidars).u = nan(length(probe(1).r),length(probe(1).phiVector));
for tt = 1:length(probe(1).phiVector)
    interpVel(1).u(:,tt) = interp3(xVector,yVector,zVector,uComp...
        ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
        ,Lidar(1).scan(tt).CartZ);
    interpVel(1).v(:,tt) = interp3(xVector,yVector,zVector,vComp...
        ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
        ,Lidar(1).scan(tt).CartZ);
    interpVel(1).w(:,tt) = interp3(xVector,yVector,zVector,wComp...
        ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
        ,Lidar(1).scan(tt).CartZ);
end

% calculate the LOS velocity of the LIDAR for each and every point of the
% scan, i.e. 153x11 points in total
LOSvel = nan(length(probe(1).r),length(probe(1).phiVector));
for tt = 1:length(probe(1).phiVector)
    LOSvel(:,tt) = [sin(probe(1).phiVector(tt)-pi/2)*cos(thetaLidar)...
        ,cos(probe(1).phiVector(tt)-pi/2)*cos(thetaLidar),sin(thetaLidar)]...
        *[interpVel(1).u(:,tt)'; interpVel(1).v(:,tt)';interpVel(1).w(:,tt)'];
end
        
        
%% Implement weighting function

% preallocate matrix with LIDAR measurements
LidarMeas = nan(probe(1).NRangeGates,length(probe(1).phiVector));

% apply the ranging function to the points of each range gate. For each
% range gate one value is returned and that corresponds to the final
% measurement that the LIDAR returns. 
for ii = 1:probe(1).NRangeGates
    for tt = 1:length(probe(1).phiVector)
        LidarMeas(ii,tt) = WeightFunc*...
            LOSvel(probe(1).Points*(ii-1)+1:probe(1).Points*ii,tt)/sum(WeightFunc);
    end
end
   
%% Find the points that the measurements correspond to 

measurPoints(length(probe(1).phiVector)).x = [];
% We already have the cartesian coordinates in Lidar.scan but it is not 
% easy to retrieve the exact points. Here the coordinates are separated 
% into points.
%
% NOTE: maybe in the future it will be easier if instead of a structure we 
% have a matrix with dimensions (length(probe(1).phiVector),3) where each 
% column will correspond to one coordinate x, y, z.

for ii = 1:probe(1).NRangeGates
    for tt = 1:length(probe(1).phiVector)
        measurPoints(tt).x = Lidar(1).scan(tt).CartX((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
        measurPoints(tt).y = Lidar(1).scan(tt).CartY((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
        measurPoints(tt).z = Lidar(1).scan(tt).CartZ((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
    end
end

%% Call SecondLidar Script only if there is a second Lidar!!

if (nLidars > 1) 
    SecondLidar
end
%% END 
