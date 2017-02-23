function ReconstrVelVector = ReconstructVelocityFunction(Lidar,Points,nLidars)
%
% PURPOSE: Reconstruct the 2D or 3D velocity at a given point depending on
% the number of Lidars.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Inputs:
% 
% 
% Outputs:
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 22, 2017
% Last edited: February 22, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pre-allocate the matrix with the reconstructed velocity components. Every
% column contains the u and v components for a single point.
ReconstrVelVector = nan(nLidars,Points);

for ii = 1:Points
    if (nLidars == 2)
    
        % Reconstruct the 2D velocity. The w-component is not taken into
        % account at all.
        A = [cos(Lidar(1).phi(ii))  sin(Lidar(1).phi(ii));...
            cos(Lidar(2).phi(ii)) sin(Lidar(2).phi(ii))];
        b = [Lidar(1).LidarMeas(ii); Lidar(2).LidarMeas(ii)];

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

        b = [Lidar(1).LidarMeas(ii); Lidar(2).LidarMeas(ii); Lidar(3).LidarMeas(ii)];
    
        % more information about how this matrix is derived can be found in 
        % paper:
        % "3D Turbulence Measurements Using Three Synchronous Wind Lidars:
        % Validation against Sonic Anemometry" by Carbajo F. et al, 2014.
        %
        % NOTE: The paper uses a different convention than MATLAB. The 
        % methodology followed here is similar to the paper but the formulas 
        % follow MATLAB conventions. More specifically, the difference is
        % located in the measuring of the azimuthal angle (see paper end of pg.
        % 1550).
        %
        % LOS = A*b, where:
        % 
        % A = [sin(phi(1))cos(theta(1)) cos(phi(1))cos(theta(1))  sin(theta(1))
        %      sin(phi(2))cos(theta(2)) cos(phi(2))cos(theta(2))  sin(theta(2))
        %      sin(phi(3))cos(theta(3)) cos(phi(3))cos(theta(3))  sin(theta(3))]
        % and 
        % 
        % b = [u
        %      v
        %      w]
        %
        % At this point we are solving for b to verify that the reconstructed
        % velocity is equal to the velocity field we have input.

        % to solve the linear system of equations we have: Ax = b =>
        % x = inv(A)*b. ( A\b is the same as inv(A)*b in Matlab).
        ReconstrVelVector(:,ii) = A\b;
    end
end


end % END of function