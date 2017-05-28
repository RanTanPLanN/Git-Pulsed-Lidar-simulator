function [phi,theta,r] = CalcPhiThetaR(LidarX,LidarY,LidarZ,measurePointX,measurePointY,measurePointZ)
%
% PURPOSE: Calculate the azimuthal angle offset, theta angle and focus 
% distance of an additional LIDAR. 
% Up to this point all the calculations are performed w.r.t. point (x,y,z) 
% = (0,0,0), i.e. LIDAR 1. Since LIDARs 2 and 3 have different locations,
% in order to face the measurement point(s), they need different inputs.
% These inputs are calculated in this function.
%
% NOTE: In PPI mode, each output is a scalar since the LIDAR focuses on 1
% point and scans around it. In staring mode each output is a vector with
% length equal to the points we want to measure.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Inputs:
%
% 1. LidarX: x-coordinate of the examined LIDAR position
% 2. LidarY: y-coordinate of the examined LIDAR position
% 3. LidarZ: z-coordinate of the examined LIDAR position
% 4. measurePointX: x-coordinate of the focus point(s).
% 5. measurePointY: y-coordinate of the focus point(s).
% 6. measurePointZ: z-coordinate of the focus point(s).
%
% %%%%
%
% Outputs:
% 
% 1. phi: The azimuthal angle difference(s) between the LOS of LIDAR 1
% and the LOS of the examined LIDAR. The returned value(s) is in degrees.
% The angles follow the MATLAB convention.
% 2. theta: The elevation angle(s)
% 3. r: Radial distance(s) from LIDAR to the focus point(s).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% Created: February 14, 2017
% Last edited: March 22, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate the distance in the xy-plane between the LIDAR and the
% measurement points. The resulting vector (hypotenuse) will have length
% equal to the number of measurement points.
hypotenuse = sqrt((LidarY - measurePointY).^2 ...
    + (LidarX - measurePointX).^2);

% calculate the necessary elevation angle theta to point at each
% measurement point
theta = atan((measurePointZ - LidarZ)./hypotenuse);

% calculate the radial distances between the LIDAR and the points
r = hypotenuse./cos(theta);

% MATLAB defines the azimuthal angle as the angle between the x-axis and
% the vector, measuring from the x-axis to the vector. Therefore, depending
% on the position of the LIDAR, an azimuthal offset must be added to the
% the phiVector in order to convert the coordinates from spherical to
% cartesian correctly.
phi = atand(abs((LidarX - measurePointX)./(LidarY - measurePointY)));

% The offset also depends on the position of the LIDAR relatively to the
% measuring point:
for ii = 1:length(phi)
    if (LidarY >= measurePointY(ii) && LidarX > measurePointX(ii))
        phi(ii) = 270 - phi(ii);
    elseif (LidarY < measurePointY(ii) && LidarX >= measurePointX(ii))
        phi(ii) = 90 + phi(ii);
    elseif (LidarY >= measurePointY(ii) && LidarX <= measurePointX(ii))
        phi(ii) = 270 + phi(ii);
    % In the ast check an 'elseif' is used instead of 'else' to include the 
    % case when we give a point in cartesian coordinates in the PPI mode
    % because then this function will be also called at the very beginning 
    % in order to calculate the phi, theta and r of LIDAR 1.
    elseif (LidarY < measurePointY(ii) && LidarX < measurePointX(ii))
        phi(ii) = 90 - phi(ii);
    end
end


end % END of function