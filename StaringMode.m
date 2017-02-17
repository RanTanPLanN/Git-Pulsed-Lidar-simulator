%% Staring mode script
%
% PURPOSE: Script to simulate the operation of the LIDARs in "staring"
% mode. Staring mode is when the LIDARs do not scan in PPI or RHI but focus
% on given points. The user will input the LIDAR location and the points of
% interest and then obtain the 2D or 3D velocity at those points. The
% points do not have to follow a certain pattern.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 13, 2017
% Last edited: February 16, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% include large for-loop to run this script for all the LIDARs
for nn = 1:nLidars
    if (nn == 1)
        % convert the points of interest from cartesian to spherical. 
        % WARNING: At this point this can only be done for the 1st Lidar because it
        % is located at (x,y,z) = (0,0,0). For the rest the process will be done
        % manually.
        [Lidar(nn).phi, Lidar(nn).theta, Lidar(nn).r] = ...
            cart2sph(CartInputPoints(:,1),CartInputPoints(:,2),CartInputPoints(:,3));
    else
        [Lidar(nn).phi,Lidar(nn).theta,Lidar(nn).r] = CalcPhiThetaR(Lidar(nn).x...
            ,Lidar(nn).y,Lidar(1).x,CartInputPoints(:,1),CartInputPoints(:,2)...
            ,CartInputPoints(:,3));

        Lidar(nn).phi = deg2rad(Lidar(nn).phi);
    end
    % In Staring mode the range gates are named 'beams' instead of 'scans' that
    % was the name in the PPI mode just to avoid confusion. Here, the LIDAR is
    % NOT scanning, rather sending a single beam (range gate) at focus point
    % in order to obtain a measurement at that point.

    % probe(nLidars).Points = [];
    % probe(nLidars).LengthDiscr = [];
    % probe(nLidars).r = [];
    for ii = 1:size(CartInputPoints,1)
        [Lidar(nn).beam(ii),probe(nn).Points,probe(nn).LengthDiscr,probe(nn).r(ii,:)]...
            = calculateRangeGates(Lidar(nn).r(ii),probe(nn).Length...
            ,probe(nn).PointsPerLength,NaN,probe(nn).FirstGap,NaN...
            ,Lidar(nn).phi(ii),Lidar(nn).theta(ii),operationMode);
    end




% % calculate the distance in the xy-plane between the LIDAR and the
% % measurement points. The resulting vector (hypotenuse) will have length
% % equal to the number of measurement points.
% hypotenuse = sqrt((Lidar(2).y-CartInputPoints(:,2)).^2 ...
%     + (Lidar(2).x-CartInputPoints(:,1)).^2);
% 
% % calculate the necessary elevation angle theta to point at each
% % measurement point
% Lidar(2).theta = atan(CartInputPoints(:,3)./hypotenuse);
% 
% % calculate the radial distances between the LIDAR and the points
% Lidar(2).r = hypotenuse./cos(Lidar(2).theta);
% 
% % MATLAB defines the azimuthal angle as the angle between the x-axis and
% % the vector, measuring from the x-axis to the vector. Therefore, depending
% % on the position of the LIDAR, an azimuthal offset must be added to the
% % the phiVector in order to convert the coordinates from spherical to
% % cartesian correctly.
% Lidar(2).phi = atand(abs((Lidar(2).x-CartInputPoints(:,1))./(Lidar(2).y-CartInputPoints(:,2))));
% 
% % The offset also depends on the position of the LIDAR relatively to the
% % measuring point:
% for ii = 1:length(Lidar(2).phi)
%     if (Lidar(2).y >= CartInputPoints(ii,2) && Lidar(2).x > Lidar(1).x)
%         Lidar(2).phi(ii) = 270 - Lidar(2).phi(ii);
%     elseif (Lidar(2).y < CartInputPoints(ii,2) && Lidar(2).x > Lidar(1).x)
%         Lidar(2).phi(ii) = 90 + Lidar(2).phi(ii);
%     elseif (Lidar(2).y >= CartInputPoints(ii,2) && Lidar(2).x < Lidar(1).x)
%         Lidar(2).phi(ii) = 270 + Lidar(2).phi(ii);
%     else
%         Lidar(2).phi(ii) = 90 - Lidar(2).phi(ii);
%     end
% end
% 

% for ii = 1:size(CartInputPoints,1)
%     [Lidar(2).beam(ii),probe(2).Points,probe(2).LengthDiscr,probe(2).r(ii,:)]...
%         = calculateRangeGates(Lidar(2).r(ii),probe(2).Length...
%         ,probe(2).PointsPerLength,NaN,probe(2).FirstGap,NaN...
%         ,Lidar(2).phi(ii),Lidar(2).theta(ii),operationMode);
% end
    if (nn > 1)
        for ii = 1:size(CartInputPoints,1);
            Lidar(nn).beam(ii).CartX = Lidar(nn).beam(ii).CartX + Lidar(nn).x;
            Lidar(nn).beam(ii).CartY = Lidar(nn).beam(ii).CartY + Lidar(nn).y;

            % the z-coordinate does not need to be adjusted since we assumed
            % ground based LIDARs (z=0). However, it is good to have in case
            % something changes.
            Lidar(nn).beam(ii).CartZ = Lidar(nn).beam(ii).CartZ + Lidar(nn).z;
        end
    end
    
    
%% Interpolate the grid points and calculate the LOS velocity

    % Call interpVel function to calculate the interpolated velocities
    [interpVel(nn)] = InterpFunc...
        (xVector,yVector,zVector,uComp,vComp,wComp,Lidar(nn).beam...
        ,Lidar(nn).phi);
    
    
    % calculate the LOS velocity of the LIDAR for each and every point 
    % of all beams, i.e. 51x4 points in total
    Lidar(nn).LOSvel = nan(probe(nn).Points,length(Lidar(nn).phi));
    for tt = 1:length(Lidar(nn).phi)
        Lidar(nn).LOSvel(:,tt) = [sin(Lidar(nn).phi(tt)-pi/2)*cos(Lidar(nn).theta(tt))...
            ,cos(Lidar(nn).phi(tt)-pi/2)*cos(Lidar(nn).theta(tt)),...
            sin(Lidar(nn).theta(tt))]*[interpVel(nn).u(:,tt)';...
            interpVel(nn).v(:,tt)';interpVel(nn).w(:,tt)'];
    end
    
%% Calculate and apply weighting function

    WeightFunc = WeightingFunction(probe(nn).LengthDiscr,weightingFuncType);
% 
%     % preallocate matrix with LIDAR measurements
% %     Lidar(nn).LidarMeas = nan(probe(nn).NRangeGates,length(probe(nn).phiVector));
% 
%     % apply the ranging function to the points of each range gate. For each
%     % range gate one value is returned and that corresponds to the final
%     % measurement that the LIDAR returns. 
% %     for ii = 1:probe(nn).NRangeGates
% %         for tt = 1:length(Lidar(nn).phi)
    Lidar(nn).LidarMeas = WeightFunc*Lidar(nn).LOSvel/sum(WeightFunc);
%         end
%     end
    
end % end of large for-loop

%% Plot the beams of all LIDARs

figure
for ii = 1:size(CartInputPoints,1)
     hold on
     
     % plot the beam of each LIDAR
     plot(Lidar(1).beam(ii).CartX, Lidar(1).beam(ii).CartY,'.')
     plot(Lidar(2).beam(ii).CartX, Lidar(2).beam(ii).CartY,'.')
%      plot(Lidar(3).beam(ii).CartX, Lidar(3).beam(ii).CartY,'.')
     
     % plot the focal points
     plot(CartInputPoints(ii,1),CartInputPoints(ii,2),'*')
     
     % plot the LIDAR position
     plot(Lidar(1).x,Lidar(1).y,'^')
     plot(Lidar(2).x,Lidar(2).y,'d')
%      plot(Lidar(3).x,Lidar(3).y,'o')
end
hold off
grid on
grid minor
xlabel('x axis')
ylabel('y axis')
title('Staring mode in xy-plane')

%% END of script