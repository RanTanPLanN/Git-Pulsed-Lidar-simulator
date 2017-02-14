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
% Last edited: February 14, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% convert the points of interest from cartesian to spherical. 
% WARNING: At this point this can only be done for the 1st Lidar because it
% is located at (x,y,z) = (0,0,0). For the rest the process will be done
% manually.
[Lidar(1).phi, Lidar(1).theta, Lidar(1).r] = ...
    cart2sph(CartInputPoints(:,1),CartInputPoints(:,2),CartInputPoints(:,3));

% In Staring mode the range gates are named 'beams' instead of 'scans' that
% was the name in the PPI mode just to avoid confusion. Here, the LIDAR is
% NOT scanning, rather sending a single beam (range gate) at focus point
% in order to obtain a measurement at that point.

% probe(nLidars).Points = [];
% probe(nLidars).LengthDiscr = [];
% probe(nLidars).r = [];
% for nn = 1:nLidars
for ii = 1:size(CartInputPoints,1)
    [Lidar(1).beam(ii),probe(1).Points,probe(1).LengthDiscr,probe(1).r(ii,:)]...
        = calculateRangeGates(Lidar(1).r(ii),probe(1).Length...
        ,probe(1).PointsPerLength,NaN,probe(1).FirstGap,NaN...
        ,Lidar(1).phi(ii),Lidar(1).theta(ii),operationMode);
end
% end

[Lidar(2).phi,Lidar(2).theta,Lidar(2).r] = CalcPhiThetaR(Lidar(2).x...
    ,Lidar(2).y,Lidar(1).x,CartInputPoints(:,1),CartInputPoints(:,2)...
    ,CartInputPoints(:,3));

Lidar(2).phi = deg2rad(Lidar(2).phi);

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

for ii = 1:size(CartInputPoints,1)
    [Lidar(2).beam(ii),probe(2).Points,probe(2).LengthDiscr,probe(2).r(ii,:)]...
        = calculateRangeGates(Lidar(2).r(ii),probe(2).Length...
        ,probe(2).PointsPerLength,NaN,probe(2).FirstGap,NaN...
        ,Lidar(2).phi(ii),Lidar(2).theta(ii),operationMode);
end

for ii = 1:size(CartInputPoints,1);
    Lidar(2).beam(ii).CartX = Lidar(2).beam(ii).CartX + Lidar(2).x;
    Lidar(2).beam(ii).CartY = Lidar(2).beam(ii).CartY + Lidar(2).y;
    
    % the z-coordinate does not need to be adjusted since we assumed
    % ground based LIDARs (z=0). However, it is good to have in case
    % something changes.
    Lidar(2).beam(ii).CartZ = Lidar(2).beam(ii).CartZ + Lidar(2).z;
end

%% Plot the beams of all LIDARs
figure
for ii = 1:size(CartInputPoints,1)
     hold on
     plot(Lidar(1).beam(ii).CartX, Lidar(1).beam(ii).CartY,'.')
     plot(CartInputPoints(ii,1),CartInputPoints(ii,2),'*')
     plot(Lidar(2).beam(ii).CartX, Lidar(2).beam(ii).CartY,'.')
end
hold off
grid on
grid minor
xlabel('x axis')
ylabel('y axis')

%% END