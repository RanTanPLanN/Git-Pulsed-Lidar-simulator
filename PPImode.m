%% (new) PPImode script
%
% PURPOSE: Simulate the operation of the LIDAR(s) in PPI mode (Plan
% Position Indicator)
%
% NOTE: PPImode is a script and not a function because it is more
% convenient to keep all the user inputs in the workspace and not have to
% change anything in this script. All the necessary changes will occur in
% 'PulsedLidarSim' script.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


% include large for-loop to run this script for all the LIDARs
for nn = 1:nLidars
    
    if (nn == 1)
        probe(1).phiVector = (-phiLidar:phiStep:phiLidar) + phiMeasurePoint;
        probe(1).phiVector = deg2rad(-probe(1).phiVector + 90);
    else
        [phiOffset,thetaLidar(nn),probe(nn).RadialDist2MeasurePoint] = ...
            CalcPhiThetaR(Lidar(nn).x,Lidar(nn).y,Lidar(1).x,...
            measurePoint.x,measurePoint.y,measurePoint.z);

        probe(nn).phiVector = -[-phiLidar:phiStep:phiLidar] + phiOffset;
        probe(nn).phiVector = deg2rad(probe(nn).phiVector);
    end
    
    % call calculateRangeGates function for all LIDARs
    [Lidar(nn).scan,probe(nn).Points,probe(nn).LengthDiscr,probe(nn).r] = ...
        calculateRangeGates(probe(nn).RadialDist2MeasurePoint,probe(nn).Length...
        ,probe(nn).PointsPerLength,probe(nn).RangeGateGap,probe(nn).FirstGap...
        ,probe(nn).NRangeGates,probe(nn).phiVector,thetaLidar(nn),operationMode);
    
    if (nn == 1)
        [measurePoint.x,measurePoint.y,measurePoint.z] = ...
            sph2cart(probe(1).phiVector((length(probe(1).phiVector)+1)/2)...
            ,thetaLidar(1),probe(1).RadialDist2MeasurePoint);
    else
        % Adjust the cartesian coordinates to the position of the 2nd LIDAR
        % otherwise all the points will rotate about point (0,0,0).
        for tt = 1:length(probe(nn).phiVector)
            Lidar(nn).scan(tt).CartX = Lidar(nn).scan(tt).CartX + Lidar(nn).x;
            Lidar(nn).scan(tt).CartY = Lidar(nn).scan(tt).CartY + Lidar(nn).y;

            % the z-coordinate does not need to be adjusted since we assumed
            % ground based LIDARs (z=0). However, it is good to have in case
            % something changes.
            Lidar(nn).scan(tt).CartZ = Lidar(nn).scan(tt).CartZ + Lidar(nn).z;
        end
    end
    
    % The following line is needed because, even though the user selects
    % the number of RGs, after the call of 'calculateRangeGates' function a
    % and if the focal point is too far away from the LIDAR, there might be
    % room for more RGs before the one that encloses the focal point,
    % therefore the final number of RGs for that LIDAR will be larger than
    % the one that the user selected. However it can never be smaller than
    % the selected number.
    % To sum up, if this command is neglected then the ending result
    % included less points measurements than the actual number of RGs.
    probe(nn).NRangeGates = size(probe(nn).r,2)/probe(nn).Points;
    
%% Plot the beam from the side and the front

    figure
    for ii = 1:length(probe(nn).phiVector)
        clf
        hold on
        subplot(1,2,1)
        plot(Lidar(nn).scan(ii).CartX, Lidar(nn).scan(ii).CartY,'.')
        ylabel('y axis')
        xlabel('x axis')
        title('Top view')
        axis([-100 150 -20 405])
        grid on

        subplot(1,2,2)
        plot(Lidar(nn).scan(ii).CartX, Lidar(nn).scan(ii).CartZ,'r.')
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
    [interpVel(nn)] = InterpFunc...
        (xVector,yVector,zVector,uComp,vComp,wComp,Lidar(nn).scan...
        ,probe(nn).phiVector);

    % calculate the LOS velocity of the LIDAR for each and every point of the
    % scan, i.e. 225x11 points in total
    Lidar(nn).LOSvel = nan(length(probe(nn).r),length(probe(nn).phiVector));
    for tt = 1:length(probe(nn).phiVector)
        Lidar(nn).LOSvel(:,tt) = [sin(probe(nn).phiVector(tt)-pi/2)*cos(thetaLidar(nn))...
            ,cos(probe(nn).phiVector(tt)-pi/2)*cos(thetaLidar(nn)),sin(thetaLidar(nn))]...
            *[interpVel(nn).u(:,tt)'; interpVel(nn).v(:,tt)';interpVel(nn).w(:,tt)'];
    end

%% Calculate and apply weighting function

    WeightFunc = WeightingFunction(probe(nn).LengthDiscr,weightingFuncType);

    % preallocate matrix with LIDAR measurements
    Lidar(nn).LidarMeas = nan(probe(nn).NRangeGates,length(probe(nn).phiVector));

    % apply the ranging function to the points of each range gate. For each
    % range gate one value is returned and that corresponds to the final
    % measurement that the LIDAR returns. 
    for ii = 1:probe(nn).NRangeGates
        for tt = 1:length(probe(nn).phiVector)
            Lidar(nn).LidarMeas(ii,tt) = WeightFunc*...
                Lidar(nn).LOSvel(probe(nn).Points*(ii-1)+1:...
                probe(nn).Points*ii,tt)/sum(WeightFunc);
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
end % end of large for-loop

%% Visualize the PPI scans of all Lidars

figure
for ii = 1:length(probe(nn).phiVector)
    clf
    subplot(1,2,1)
    hold on
    plot(Lidar(1).scan(ii).CartX, Lidar(1).scan(ii).CartY,'.')
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartY,'.')
    plot(Lidar(3).scan(ii).CartX, Lidar(3).scan(ii).CartY,'.')

    ylabel('y axis')
    xlabel('x axis')
    title('Top view')
    axis([-100 150 -20 405])
    grid on

    subplot(1,2,2)
    hold on
    plot(Lidar(1).scan(ii).CartX, Lidar(1).scan(ii).CartZ,'.')
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartZ,'.')
    plot(Lidar(3).scan(ii).CartX, Lidar(3).scan(ii).CartZ,'.')

    ylabel('z axis')
    xlabel('x axis')
    axis([-100 100 18 80 ])
    title('Front view')
    grid on
    drawnow
end
hold off

%% END of script