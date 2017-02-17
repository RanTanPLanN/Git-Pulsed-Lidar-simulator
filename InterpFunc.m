function [interpVel] = InterpFunc(xVector,yVector,zVector,uComp,vComp,wComp,scan,phiVector)
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
% 8. phiVector: vector with the azimuthal angles the LIDAR is scanning. The
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
% Last edited: February 16, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pre-allocate structure
interpVel(1).u = nan(length(scan(1).CartX),length(phiVector));

for tt = 1:length(phiVector)
    
    % each column of the u,v,w matrices corresponds to a different phi
    % angle. 
    % In PPI mode the columns will be equal to the azimuthal
    % resolution of the LIDARs (e.g. if phiVector = [-25:5:25] then there
    % will be 11 columns.
    % In Staring mode, there is one phi per point,
    % therefore the columns will be equal to the number of points.
    
    interpVel(1).u(:,tt) = interp3(xVector,yVector,zVector,uComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
    interpVel(1).v(:,tt) = interp3(xVector,yVector,zVector,vComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
    interpVel(1).w(:,tt) = interp3(xVector,yVector,zVector,wComp...
        ,scan(tt).CartX,scan(tt).CartY,scan(tt).CartZ);
end

end % END of function