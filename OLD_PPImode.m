%% OLD PPImode script
%
% PURPOSE: Simulate the operation of the LIDAR(s) in PPI mode (Plan
% Position Indicator)
%
% NOTE: PPImode is a script and not a function because it is more
% convenient to keep all the user inputs in the workspace and not have to
% change anything in this script. All the necessary changes will occur in
% 'PulsedLidarSim' script.
%
% UPDATE February 14: This script is not used anymore. The only thing that
% could be used from here is some plots that were (most of the time)
% commented out.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 9, 2017
% Last edited: February 14, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Probe 

% Create vector of azimuthial angles and convert it to radians. I have to
% add 90 degrees because of the way MATLAB converts spherical to cartesian 
% coordinates. For more info see:
% https://www.mathworks.com/help/matlab/ref/sph2cart.html .
probe(1).phiVector = (-phiLidar:phiStep:phiLidar) + phiMeasurePoint;
probe(1).phiVector = deg2rad(-probe(1).phiVector + 90);

[Lidar(1).scan,probe(1).Points,probe(1).LengthDiscr,probe(1).r] = ...
    calculateRangeGates(probe(1).RadialDist2MeasurePoint,probe(1).Length...
    ,probe(1).PointsPerLength,probe(1).RangeGateGap,probe(1).FirstGap...
    ,probe(1).NRangeGates,probe(1).phiVector,thetaLidar(1),operationMode);

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

% condX = Lidar(1).scan(1).CartX(end)>=xVector;
% differX = diff(condX);
% 
% % maybe this method is faster, i dont know yet.
% Xlow = xVector(logical(abs(differX)));
% Xhigh = xVector(logical([0 abs(differX)]));
% CubeX = [Xlow Xhigh];
% 
% condY = Lidar(1).scan(1).CartY(end)>=yVector;
% differY = diff(condY);
% Ylow = yVector(logical(abs(differY)));
% Yhigh = yVector(logical([0 abs(differY)]));
% CubeY = [Ylow Yhigh];
% 
% condZ = Lidar(1).scan(1).CartZ(end)>=zVector;
% differZ = diff(condZ);
% Zlow = zVector(logical(abs(differZ)));
% Zhigh = zVector(logical([0 abs(differZ)]));
% CubeZ = [Zlow Zhigh];
% 
% [cube.x,cube.y,cube.z] = meshgrid(CubeX, CubeY, CubeZ);

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
% interpVel(1).u = nan(length(probe(1).r),length(probe(1).phiVector));
% for tt = 1:length(probe(1).phiVector)
%     interpVel(1).u(:,tt) = interp3(xVector,yVector,zVector,uComp...
%         ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
%         ,Lidar(1).scan(tt).CartZ);
%     interpVel(1).v(:,tt) = interp3(xVector,yVector,zVector,vComp...
%         ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
%         ,Lidar(1).scan(tt).CartZ);
%     interpVel(1).w(:,tt) = interp3(xVector,yVector,zVector,wComp...
%         ,Lidar(1).scan(tt).CartX,Lidar(1).scan(tt).CartY...
%         ,Lidar(1).scan(tt).CartZ);
% end

% Call interpVel function to calculate the interpolated velocities
[interpVel(1)] = InterpFunc...
    (xVector,yVector,zVector,uComp,vComp,wComp,Lidar(1).scan,probe(1).r,probe(1).phiVector);

% calculate the LOS velocity of the LIDAR for each and every point of the
% scan, i.e. 153x11 points in total
Lidar(1).LOSvel = nan(length(probe(1).r),length(probe(1).phiVector));
for tt = 1:length(probe(1).phiVector)
    Lidar(1).LOSvel(:,tt) = [sin(probe(1).phiVector(tt)-pi/2)*cos(thetaLidar(1))...
        ,cos(probe(1).phiVector(tt)-pi/2)*cos(thetaLidar(1)),sin(thetaLidar(1))]...
        *[interpVel(1).u(:,tt)'; interpVel(1).v(:,tt)';interpVel(1).w(:,tt)'];
end
        
        
%% Calculate and apply weighting function

WeightFunc = WeightingFunction(probe(1).LengthDiscr,weightingFuncType);

% plot normalized weighting function
figure
plot(probe(1).LengthDiscr,WeightFunc)
hold on
plot([probe(1).LengthDiscr((end+1)/2) probe(1).LengthDiscr((end+1)/2)]...
    ,ylim,'k--')
hold off
xlabel('Probe length [m]')
ylabel('PDF')
title('Weighting function')

% preallocate matrix with LIDAR measurements
Lidar(1).LidarMeas = nan(probe(1).NRangeGates,length(probe(1).phiVector));

% apply the ranging function to the points of each range gate. For each
% range gate one value is returned and that corresponds to the final
% measurement that the LIDAR returns. 
for ii = 1:probe(1).NRangeGates
    for tt = 1:length(probe(1).phiVector)
        Lidar(1).LidarMeas(ii,tt) = WeightFunc*...
            Lidar(1).LOSvel(probe(1).Points*(ii-1)+1:probe(1).Points*ii,tt)/sum(WeightFunc);
    end
end
   
%% Find the points that the measurements correspond to 

probe(1).measurPoints(length(probe(1).phiVector)).x = [];
% We already have the cartesian coordinates in Lidar.scan but it is not 
% easy to retrieve the exact points. Here the coordinates are separated 
% into points.
%
% NOTE: maybe in the future it will be easier if instead of a structure we 
% have a matrix with dimensions (length(probe(1).phiVector),3) where each 
% column will correspond to one coordinate x, y, z.

% The structure of the probe(1).measurPoints is as follows at the moment:
% Each row contains 3 x-coordinates, 3 y-coordinates and 3 z-coordinates.
% These correspond to the middle points of the 3 range gates for the first
% phi angle of the scanning plane. The next row has the coordinates of the
% 3 middle points of of the 2 range gates for the second phi angle and so
% on.
for ii = 1:probe(1).NRangeGates
    for tt = 1:length(probe(1).phiVector)
        probe(1).measurPoints(tt).x =...
            Lidar(1).scan(tt).CartX((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
        probe(1).measurPoints(tt).y =...
            Lidar(1).scan(tt).CartY((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
        probe(1).measurPoints(tt).z =...
            Lidar(1).scan(tt).CartZ((probe(1).Points+1)/2:probe(1).Points:length(probe(1).r));
    end
end

%% Call SecondLidar Script only if there is a second Lidar!!

if (nLidars > 1) 
    AdditionalLidars
% else
%     reconstruct velocity here
end
% here there should be an 'else' statement that if there is not any other
% LIDARs the 1D velocity should be reconstructed at this point.
%% END 

