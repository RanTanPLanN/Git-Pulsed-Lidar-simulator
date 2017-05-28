function WeightFunc = WeightingFunction(probeLengthVec, FunctionShape)
%
% PURPOSE: Create the range weighting function for a pulsed LIDAR. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%
% 1. probeLengthVec : Vector. The discretized length of the probe into
% probe.Points (see main script).
% 2. FunctionShape: Char. Defines the shape of the weighting function. If 
% FunctionShape = 't' then the function is triangular, else if 
% FunctionShape = 'g' it is Gaussian.
%
% Outputs:
%
% 1. WeightFunc: A vector with the discretized range weighting function. 
% This will be applied on the probe points to obtain a single measurement.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUTURE WORK: Develop a generic range weighting function both for pulsed
% and CW lidar.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 2nd, 2017
% Last edited: February 6, 2017
% Author: Nikolaos Frouzakis

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

probePoints = length(probeLengthVec);
probeLength = probeLengthVec(end);

switch FunctionShape
    case 't'
        % The function will have the shape of an isosceles triangle and 
        % will have as many points as probePoints. Top point of the 
        % triangle - maximum weight point (the 30 degrees angles 
        % was chosen arbitrarilly because it doens matter since we will
        % normalize later):
        maxWeight = probeLength/2*tand(30);

        % Range weighting function
        WeightFunc = [linspace(0,maxWeight,(probePoints+1)/2)];
        WeightFunc = [WeightFunc(1:end-1) maxWeight WeightFunc(end-1:-1:1)];

        % normalize the weighting function so that the area is 1
        Area = probeLength*maxWeight/2;
        
    case 'g'
        % use Dr and Dp from the NREL report
        Dr = probeLength;
        Dp = probeLength;
        
        % width of the pulse (rp^2 = 2*sigma^2).
        rp = Dr/(2*log(2)^0.5);
        
        % Center of the Range Gate. Equivalent to the
        % mean of the Gaussian distribution
        R = probeLengthVec((probePoints + 1)/2);
        
        WeightFunc = 1/(sqrt(pi)*rp)*exp(-(R - probeLengthVec).^2/rp^2);
        Area = trapz(probeLengthVec,WeightFunc);
end % end of switch

% normalize pdf
WeightFunc = WeightFunc/Area;

end % end of function