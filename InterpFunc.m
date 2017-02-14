function [interpVel] = InterpFunc(xVector,yVector,zVector,uComp,vComp,wComp,scan,r,phiVector)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PURPOSE: Interpolate the velocities from the LES grid to the points of
% the LIDAR beam. 
%
% NOTE: Not something very sophisticated is happening in this function. The
% main reason it was created it was to keep the code as clean and
% structured as possible.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%
% 1. xVector: vector with the x-coordinates of the grid
% 2. yVector: vector with the y-coordinates of the grid
% 3. zVector: vector with the z-coordinates of the grid
% 4. uComp: 3D matrix with the u-component of the wind velocity at every
% grid point.
% 5. vComp: 3D matrix with the v-component of the wind velocity at every
% grid point.
% 6. wComp: 3D matrix with the w-component of the wind velocity at every
% grid point.
% 7. scan: structure with all the cartesian coordinates of all the scanning
% points of the LIDAR
% 8. r: vector with all the radial distances the LIDAR is scanning
% 9. phiVector: vector with the azimuthal angles the LIDAR is scanning. The
% angles are measured in MATLAB convention.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% OUTPUTS:
% 
% 1. interpVel: structure containing the u,v,w-components of the wind 
% velocity at the measuring points.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 9, 2017
% Last edited: February 14, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pre-allocate structure
interpVel(1).u = nan(length(r),length(phiVector));

for tt = 1:length(phiVector)
    interpVel(1).u(:,tt) = interp3(xVector,yVector,zVector,uComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
    interpVel(1).v(:,tt) = interp3(xVector,yVector,zVector,vComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
    interpVel(1).w(:,tt) = interp3(xVector,yVector,zVector,wComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
end

end % END of function