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
% Last edited: February 23, 2017
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
    % In Staring mode the Range Gates are named 'beams' instead of 'scans'
    % that was the name in the PPI mode just to avoid confusion. Here, the
    % LIDAR is NOT scanning, rather sending a single beam (range gate) at 
    % the focal points to obtain a measurement at those points.

    % probe(nLidars).Points = [];
    % probe(nLidars).LengthDiscr = [];
    % probe(nLidars).r = [];
    for ii = 1:size(CartInputPoints,1)
        [Lidar(nn).beam(ii),probe(nn).Points,probe(nn).LengthDiscr,probe(nn).r(ii,:)]...
            = calculateRangeGates(Lidar(nn).r(ii),probe(nn).Length...
            ,probe(nn).PointsPerLength,NaN,probe(nn).FirstGap,NaN...
            ,Lidar(nn).phi(ii),Lidar(nn).theta(ii),operationMode);
    end
    
    % since the cartesian coordinates of Lidars 2 and 3 are calculated
    % w.r.t point (0,0,0), the beam coordinates have to be shifted
    % according to the coordinates of the respective Lidar.
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
        
        % Formulas obtained from "sph2cart" MATLAB help page.
        Lidar(nn).LOSvel(:,tt) = [cos(Lidar(nn).phi(tt))*...
            cos(Lidar(nn).theta(tt)), sin(Lidar(nn).phi(tt))*...
            cos(Lidar(nn).theta(tt)), sin(Lidar(nn).theta(tt))]*...
            [interpVel(nn).u(:,tt)';interpVel(nn).v(:,tt)';interpVel(nn).w(:,tt)'];
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

%% Reconstruct 2D velocity vector

% call ReconstrcuctionVelocityFunction to reconstruct the velocity at
% each point
ReconstrVelVector = ReconstructVelocityFunction(Lidar,...
        size(CartInputPoints,1),nLidars);

% calculate the magnitude of the HORIZONTAL wind velocity vector
HorMagnitude = sqrt(sum(ReconstrVelVector(1:2,:).^2));

% calculate wind direction in xy-plane. 
% WARNING: This is only the horizontal wind direction (it is derived only
% from the u- and v-compoents of the wind speed.
HorWindDir = atan2d(ReconstrVelVector(2,:),ReconstrVelVector(1,:));

% convert the wind direction so that North is at 0 degrees and it increases
% clockwise.
% NOTE: North (or 0 degrees) coincides with the positive y-axis.
% WARNING: The following 2 lines of code should stay in that order
% otherwise the HorWindDir is first increased and therefore fulfills the
% second condition as well.
HorWindDir(HorWindDir>=90) = 450 - HorWindDir(HorWindDir>=90);
HorWindDir(HorWindDir<90) = 90 - HorWindDir(HorWindDir<90);

%% Plot the beams of all LIDARs at the 1st point
% 
% figure
% plot3(Lidar(1).beam(1).CartX,Lidar(1).beam(1).CartY, Lidar(1).beam(1).CartZ,'.')
% hold on
% plot3(Lidar(2).beam(1).CartX,Lidar(2).beam(1).CartY, Lidar(2).beam(1).CartZ,'.')
% plot3(Lidar(3).beam(1).CartX,Lidar(3).beam(1).CartY, Lidar(3).beam(1).CartZ,'.')
% plot3([CartInputPoints(1,1) umag+CartInputPoints(1,1)],...
%     [CartInputPoints(1,2) CartInputPoints(1,2)],...
%     [CartInputPoints(1,3) CartInputPoints(1,3)])
% plot3([CartInputPoints(1,1) CartInputPoints(1,1)],...
%     [CartInputPoints(1,2) vmag+CartInputPoints(1,2)],...
%     [CartInputPoints(1,3) CartInputPoints(1,3)])
% plot3([CartInputPoints(1,1) CartInputPoints(1,1)],...
%     [CartInputPoints(1,2) CartInputPoints(1,2)],...
%     [CartInputPoints(1,3) wmag+CartInputPoints(1,3)])
% hold off
% grid on
% grid minor
% axis([-40 100 50 170 20 50])
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')
% legend('beam 1', 'beam 2','beam 3','u','v','w')

%% Plot the beams of all Lidars at all focal points

figure
for ii = 1:size(CartInputPoints,1)
     hold on
     
     % plot the beam of each LIDAR
     plot(Lidar(1).beam(ii).CartX, Lidar(1).beam(ii).CartY,'.')
     plot(Lidar(2).beam(ii).CartX, Lidar(2).beam(ii).CartY,'.')
     plot(Lidar(3).beam(ii).CartX, Lidar(3).beam(ii).CartY,'.')
     
     % plot the focal points
     plot(CartInputPoints(ii,1),CartInputPoints(ii,2),'*')
     
     % plot the LIDAR position
     plot(Lidar(1).x,Lidar(1).y,'^')
     plot(Lidar(2).x,Lidar(2).y,'d')
     plot(Lidar(3).x,Lidar(3).y,'o')
end
hold off
grid on
grid minor
xlabel('x axis')
ylabel('y axis')
title('Staring mode in xy-plane')

%% END of script