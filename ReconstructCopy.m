function [ReconstrVel] = ReconstructCopy(Lidar,Points,nLidars,timeseries)
%
% PURPOSE: Reconstruct the 2D or 3D velocity at a given point depending on
% the number of Lidars.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Inputs:
% 
% 1. Lidar: Structure with the information of all Lidars involved.
% 2. Points: Scalar equal to the number of focal points
% 3. nLidars: The number of Lidars involved
% 4. timeseries: The vector with all the time period we want to reconstruct
% 
% Outputs:
% 
% 1. ReconstrVelVector: Matrix that includes the 3D (or 2D) recontructed
% velocity for all the focal points. The matrix has dimensions [m x n]
% where m is the number of velocity that are reconstructed (3 or 2) and n
% is the number of focal points
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 22, 2017
% Last edited: February 25, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pre-allocate the matrix with the reconstructed velocity components. Every
% column contains the u and v components for a single point.
% ReconstrVelVector = nan(nLidars,Points);

% have a small loop to reshape the Lidar.LidarMeas variable. LidarMeas is a
% structure of size [1 x Points]. There are 6001 such structures in each
% Lidar structure, hence as many as the time steps of the final time
% series. I want to make those into matrices so that it is faster to
% manipulate.
for nn = 1:nLidars
   % the first row of the matrix corresponds to the radial velocity
   % measured at the first point, the second to the radial velocity
   % measured at the second point etc. Each column corresponds to one time
   % step. So the final MeasMatrix (for each Lidar) should be 
   % [Points x length(timeseries)]
   Lidar(nn).MeasMatrix = reshape([Lidar(nn).time.LidarMeas],...
       [Points length(timeseries)]);  
end

% pre-allocate memory for matrices
A = nan(nLidars);
b = nan(nLidars,length(timeseries));
ReconstrVel(Points).Point = [];

for ii = 1:Points
    if (nLidars == 2)
    
        % Reconstruct the 2D velocity. The w-component is not taken into
        % account at all.
        A = [cos(Lidar(1).theta(ii))*cos(Lidar(1).phi(ii))...
            cos(Lidar(1).theta(ii))*sin(Lidar(1).phi(ii));...
            cos(Lidar(2).theta(ii))*cos(Lidar(2).phi(ii))...
            cos(Lidar(2).theta(ii))*sin(Lidar(2).phi(ii))];
%         b = [Lidar(1).LidarMeas(ii); Lidar(2).LidarMeas(ii)];
        b = [Lidar(1).MeasMatrix(ii,:); Lidar(2).MeasMatrix(ii,:)];

    else
    
        % Reconstruct the 3D velocity using all 3 LOS velocities
        A = [cos(Lidar(1).theta(ii))*cos(Lidar(1).phi(ii)),...
             cos(Lidar(1).theta(ii))*sin(Lidar(1).phi(ii)),...
             sin(Lidar(1).theta(ii));...
             cos(Lidar(2).theta(ii))*cos(Lidar(2).phi(ii)),...
             cos(Lidar(2).theta(ii))*sin(Lidar(2).phi(ii)),...
             sin(Lidar(2).theta(ii));...
             cos(Lidar(3).theta(ii))*cos(Lidar(3).phi(ii)),...
             cos(Lidar(3).theta(ii))*sin(Lidar(3).phi(ii)),...
             sin(Lidar(3).theta(ii))];

%         b = [Lidar(1).LidarMeas(ii); Lidar(2).LidarMeas(ii); Lidar(3).LidarMeas(ii)];
        b = [Lidar(1).MeasMatrix(ii,:); Lidar(2).MeasMatrix(ii,:);...
            Lidar(3).MeasMatrix(ii,:)];
    
        % more information about how this matrix is derived can be found in 
        % paper:
        % "3D Turbulence Measurements Using Three Synchronous Wind Lidars:
        % Validation against Sonic Anemometry" by Carbajo F. et al, 2014.
        %
        % NOTE: The paper uses a different angle conventions than MATLAB. The 
        % methodology followed here is similar to the paper but the formulas 
        % follow MATLAB conventions. More specifically, the difference is
        % located in the measuring of the azimuthal angle (see paper end of pg.
        % 1550).
        %
        % LOS = A*x, where:
        % 
        % A = [sin(phi(1))cos(theta(1)) cos(phi(1))cos(theta(1))  sin(theta(1))
        %      sin(phi(2))cos(theta(2)) cos(phi(2))cos(theta(2))  sin(theta(2))
        %      sin(phi(3))cos(theta(3)) cos(phi(3))cos(theta(3))  sin(theta(3))]
        % and 
        % 
        % x = [u
        %      v
        %      w]
        %
        % At this point we are solving for x to verify that the reconstructed
        % velocity is equal to the velocity field we have input.

    end
    % to solve the linear system of equations we have: Ax = b =>
    % x = inv(A)*b. (A\b is the same as inv(A)*b in Matlab).
%     ReconstrVelVector(:,ii) = A\b;
    
    % each cell of the structure corresponds to the u,v,w (or u,v if there
    % are only 2 Lidars) components of 1 point.
    ReconstrVel(ii).Point = A\b;
end


end % END of function