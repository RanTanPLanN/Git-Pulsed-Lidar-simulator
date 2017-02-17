%% printLidarInfo script
%
% PURPOSE: Print information about the user selection in the command
% window. This script serves into keeping the code clean and not have to
% write the same lines of code twice. It is chosen to be a script and not a
% function because it would be a useless effort to import all the necessary
% inputs to a function that just prints information.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Print user selections

disp('-------- Pulsed LIDAR simulator----------')
disp('-----------------------------------------')
disp(' ')
fprintf('Number of LIDARs: %d\n', nLidars) 
if operationMode == 'p'
    disp('Operation mode: PPI')
else
    disp('Operation mode: Staring')
end

disp(' ')
disp(' ')
% Print all user choices in the command window
disp('------Information about 1st LIDAR--------')
disp(' ')

fprintf('Lidar Position:\n\n')
fprintf('x = %f\n',Lidar(1).x)
fprintf('y = %f\n',Lidar(1).y)
fprintf('z = %f\n\n',Lidar(1).z)

if operationMode == 'p'
    fprintf('Radial distance from point of interest: %.1f metres\n'...
        ,probe(1).RadialDist2MeasurePoint)

    fprintf('Inclination angle: %.1f degrees.\n'...
        ,rad2deg(thetaLidar(1)))
    fprintf('Azimuthial range: %.1f to %.1f degrees\n',-phiLidar,phiLidar)
    fprintf('Azimuthial step: %.1f degrees\n', phiStep)
    fprintf('Number of Range Gates: %d \n\n',probe(1).NRangeGates)
else
    fprintf('Number of focus points: %d\n\n', size(CartInputPoints,1))
end

fprintf('Closest possible range: %.1f metres\n',probe(1).FirstGap);
fprintf('Probe length: %.1f metres\n',probe(1).Length)
fprintf('Points per unit length of the probe: %d\n',probe(1).PointsPerLength)

Shape = (weightingFuncType == 't')*'triangular' +...
    (weightingFuncType == 'g')*' Gaussian ';
fprintf('Shape of the weighting function: %s\n\n', Shape)

if (nLidars > 1)
    disp('------Information about 2nd LIDAR-------')
    disp(' ')
    fprintf('Lidar Position:\n\n')
    fprintf('x = %f\n',Lidar(2).x)
    fprintf('y = %f\n',Lidar(2).y)
    fprintf('z = %f\n\n',Lidar(2).z)
    fprintf('Closest possible range: %.1f metres\n',probe(2).FirstGap);
    fprintf('Probe length: %.1f metres\n',probe(2).Length)
    fprintf('Points per unit length of the probe: %d\n',probe(2).PointsPerLength)
    fprintf('Shape of the weighting function: %s\n\n', Shape)
end
clear Shape

if operationMode == 's'
    for ii = 1:nLidars
        if sum(Lidar(ii).r < probe(ii).FirstGap) > 0
            disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            fprintf('WARNING: There are points that are too close to LIDAR %d\n',ii);
            disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        end
    end
else
    for ii = 1:nLidars
        if (probe(ii).RadialDist2MeasurePoint <= probe(ii).FirstGap + probe(ii).Length/2)
            disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            fprintf('WARNING: Measurement point too close to LIDAR %d.\n',ii)
            disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')         
        end
    end
end

disp('-------------END OF PRINT----------------')
disp('-----------------------------------------')
disp(' ')
disp(' ')
% end of printing information
%% END of script