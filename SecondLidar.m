% function   = SecondLidar()

% PURPOSE: Implement a second LIDAR in a measurement campaign in order to
% reconstruct the 2D wind speed at a certain point. The second LIDAR will
% also aim at the measurement point of the first LIDAR.



% FirstGap, PointsPerLength and NRgates will remain the same.
probe(2).FirstGap = probe(1).FirstGap;
probe(2).PointsPerLength = probe(1).PointsPerLength;
probe(2).NRangeGates = probe(1).NRangeGates;

% calculate the necessary theta angle and radial distance from LIDAR 2 to
% the measurement point. The azimuthal angle will remain the same as in the
% first LIDAR.
hypotenuse = sqrt(measurePoint.y^2 + Lidar(2).x^2);

thetaLidar(2) = atan(measurePoint.z/hypotenuse);
probe(2).RadialDist2MeasurePoint = hypotenuse/cos(thetaLidar(2));



phiOffset = atand(abs(measurePoint.y/Lidar(2).x));
probe(2).phiVector = -[-phiLidar:phiStep:phiLidar] + phiOffset;
probe(2).phiVector = deg2rad(probe(2).phiVector);

[Lidar(2).scan,probe(2).Points,probe(2).LengthDiscr,probe(2).r] = ...
    calculateRangeGates(probe(2).RadialDist2MeasurePoint,probe(2).Length, probe(2).PointsPerLength,...
    probe(2).RangeGateGap,probe(2).FirstGap,probe(2).NRangeGates,probe(2).phiVector,thetaLidar(2));

for tt = 1:length(probe(2).phiVector);
    Lidar(2).scan(tt).CartX = -Lidar(2).scan(tt).CartX + Lidar(2).x;
    Lidar(2).scan(tt).CartY = yDimension - Lidar(2).scan(tt).CartY;
end

% %%
% % Number points that form the probe
% probe(2).Points = probe(2).Length*probe(2).PointsPerLength;
% 
% % probePoints should be an odd number in order to fit a proper triangular
% % weighting function
% probe(2).Points = (~mod(probe(2).Points,2))*(probe(2).Points + 1) +...
%     mod(probe(2).Points,2)*probe(2).Points;
% 
% % Discretize the probeLength
% probe(2).LengthDiscr = linspace(0,probe(2).Length,probe(2).Points);
% 
% % Number of range gates that fit before the measurement point
% probe(2).CloseRG = floor((probe(2).RadialDist2MeasurePoint - ...
%     0.5*probe(2).Length - probe(2).FirstGap)/...
%     (probe(2).RangeGateGap + probe(2).Length));
% if (probe(2).CloseRG < 0) probe(2).CloseRG = 0; end;
% 
% % radial distance vector
% probe(2).r = probe(2).RadialDist2MeasurePoint - probe(2).Length/2 + ...
%     probe(2).LengthDiscr;
% 
% % previousProbe = probe.r-probe.RangeGateGap-probe.Length;
% previousProbe = probe(2).r;
% for ii = 1:probe(2).CloseRG
%     previousProbe = previousProbe - probe(2).RangeGateGap - probe(2).Length;
%     probe(2).r = [previousProbe, probe(2).r];
% end
% 
% % Calculate the radial distances of all the points of all the Range Gates
% for ii = 1:probe(2).NRangeGates - probe(2).CloseRG - 1
%     % calculate next Range Gate
%     nextRG = probe(2).r(end) + probe(2).RangeGateGap + probe(2).LengthDiscr;
%     probe(2).r = [probe(2).r, nextRG];
% end


%% Plot the beam

figure
for ii = 1:length(probe(2).phiVector)
    clf
    hold on
    subplot(1,2,1)
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartY,'.')
%     plot(Lidar(2).x,Lidar(2).y,'*')
    ylabel('y axis')
    xlabel('x axis')
    title('Top view')
    axis([-100 100 50 305])
    grid on
    
    subplot(1,2,2)
    plot(Lidar(2).scan(ii).CartX, Lidar(2).scan(ii).CartZ,'r.')
    ylabel('z axis')
    xlabel('x axis')
    axis([-100 100 18 70 ])
    title('Front view')
    grid on
    drawnow
end
hold off

%% END