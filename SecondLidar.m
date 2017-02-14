%% SecondLidar script
%
%%%%%
%
% PURPOSE: Implement a second LIDAR in a measurement campaign in order to
% reconstruct the 2D wind speed at a certain point. The second LIDAR will
% also aim at the measurement point of the first LIDAR.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 9, 2017
% Last edited: February 14, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 


% % calculate the necessary theta angle and radial distance from LIDAR 2 to
% % the measurement point. The azimuthal angle will remain the same as in the
% % first LIDAR.
% hypotenuse = sqrt((Lidar(2).y-measurePoint.y)^2 ...
%     + (Lidar(2).x-measurePoint.x)^2);
% 
% thetaLidar(2) = atan(measurePoint.z/hypotenuse);
% probe(2).RadialDist2MeasurePoint = hypotenuse/cos(thetaLidar(2));
% 
% % MATLAB defines the azimuthal angle as the angle between the x-axis and
% % the vector, measuring from the x-axis to the vector. Therefore, depending
% % on the position of the LIDAR, an azimuthal offset must be added to the
% % the phiVector in order to convert the coordinates from spherical to
% % cartesian correctly.
% phiOffset = atand(abs((Lidar(2).x-measurePoint.x)/(Lidar(2).y-measurePoint.y)));
% 
% % The offset also depends on the position of the LIDAR relatively to the
% % measuring point:
% if (Lidar(2).y >= measurePoint.y && Lidar(2).x > Lidar(1).x)
%     phiOffset = 270 - phiOffset;
% elseif (Lidar(2).y < measurePoint.y && Lidar(2).x > Lidar(1).x)
%     phiOffset = 90 + phiOffset;
% elseif (Lidar(2).y >= measurePoint.y && Lidar(2).x < Lidar(1).x)
%     phiOffset = 270 + phiOffset;
% else
%     phiOffset = 90 - phiOffset;
% end

[phiOffset,thetaLidar(2),probe(2).RadialDist2MeasurePoint] = CalcPhiThetaR(Lidar(2).x...
    ,Lidar(2).y,Lidar(1).x,measurePoint.x,measurePoint.y,measurePoint.z);

probe(2).phiVector = -[-phiLidar:phiStep:phiLidar] + phiOffset;
probe(2).phiVector = deg2rad(probe(2).phiVector);

[Lidar(2).scan,probe(2).Points,probe(2).LengthDiscr,probe(2).r] = ...
    calculateRangeGates(probe(2).RadialDist2MeasurePoint,probe(2).Length...
    ,probe(2).PointsPerLength,probe(2).RangeGateGap,probe(2).FirstGap...
    ,probe(2).NRangeGates,probe(2).phiVector,thetaLidar(2),operationMode);

% Adjust the cartesian coordinates to the position of the 2nd LIDAR
% otherwise all the points will rotate about point (0,0,0).
for tt = 1:length(probe(2).phiVector);
    Lidar(2).scan(tt).CartX = Lidar(2).scan(tt).CartX + Lidar(2).x;
    Lidar(2).scan(tt).CartY = Lidar(2).scan(tt).CartY + Lidar(2).y;
    
    % the z-coordinate does not need to be adjusted since we assumed
    % ground based LIDARs (z=0). However, it is good to have in case
    % something changes.
    Lidar(2).scan(tt).CartZ = Lidar(2).scan(tt).CartZ + Lidar(2).z;
end

%% Plot the beam from the side and the front

figure
for ii = 1:length(probe(2).phiVector)
    clf
    hold on
    subplot(1,2,1)
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartY,'.')
    ylabel('y axis')
    xlabel('x axis')
    title('Top view')
    axis([-100 150 -20 405])
    grid on
    
    subplot(1,2,2)
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartZ,'r.')
    ylabel('z axis')
    xlabel('x axis')
    axis([-100 100 18 80 ])
    title('Front view')
    grid on
    drawnow
end
hold off

%% Interpolate the grid points and calculate the LOS velocity

% Call interpVel function to calculate the interpolated velocities
[interpVel(2)] = InterpFunc...
    (xVector,yVector,zVector,uComp,vComp,wComp,Lidar(2).scan,probe(2).r,probe(2).phiVector);

% calculate the LOS velocity of the LIDAR for each and every point of the
% scan, i.e. 225x11 points in total
Lidar(2).LOSvel = nan(length(probe(2).r),length(probe(2).phiVector));
for tt = 1:length(probe(2).phiVector)
    Lidar(2).LOSvel(:,tt) = [sin(probe(2).phiVector(tt)-pi/2)*cos(thetaLidar(2))...
        ,cos(probe(2).phiVector(tt)-pi/2)*cos(thetaLidar(2)),sin(thetaLidar(2))]...
        *[interpVel(2).u(:,tt)'; interpVel(2).v(:,tt)';interpVel(2).w(:,tt)'];
end

%% Calculate and apply weighting function

WeightFunc = WeightingFunction(probe(2).LengthDiscr,weightingFuncType);

% preallocate matrix with LIDAR measurements
Lidar(2).LidarMeas = nan(probe(2).NRangeGates,length(probe(2).phiVector));

% apply the ranging function to the points of each range gate. For each
% range gate one value is returned and that corresponds to the final
% measurement that the LIDAR returns. 
for ii = 1:probe(2).NRangeGates
    for tt = 1:length(probe(2).phiVector)
        Lidar(2).LidarMeas(ii,tt) = WeightFunc*...
            Lidar(2).LOSvel(probe(2).Points*(ii-1)+1:probe(2).Points*ii,tt)/sum(WeightFunc);
    end
end

%% Carefull too many figures popping out
% for ii = 1:length(probe(1).phiVector)
%     figure
%     plot3(Lidar(1).scan(ii).CartX, Lidar(1).scan(ii).CartY, Lidar(1).scan(ii).CartZ);
%     hold on
%     plot3(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartY, Lidar(2).scan(ii).CartZ);
%     plot3(measurePoint.x,measurePoint.y,measurePoint.z,'*')
%     grid on
%     xlabel('x axis')
%     ylabel('y axis')
%     zlabel('z axis')
%     axis([-50 50 30 105 18 70 ])
%     hold off
% end

%% Reconstruct 2D velocity 

%% END