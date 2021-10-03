function M = currentSwingWaypoints(puntoDes,puntoCorr,dist)
%CURRENTSWINGWAYPOINTS generates additional waypoints to be used when the
% vehicle misaligns from the desired trajectory
%
%   -puntoDes: desired return point onto the trajectory; 1x3 [m m m]
%   -puntoCorr: estimated current location; 1x3 [m m m]
%   -dist: distance between waypoints in the direction of realignment; [m]

    distN = puntoCorr(1) - puntoDes(1); %Distance from starting and final point onto the North axis
    distE = puntoCorr(2) - puntoDes(2); %Distance from starting and final point onto the East axis

    %Number of waypoints to generate
    p = floor(abs(distE)/dist);
    
    deltaE = distE/p; %Distance between two waypoints onto the North axis
    deltaN = distN/p; %Distance between two waypoints onto the East axis
   
    matrix = zeros(p-1,3);
   
    %Ending at p-1 because the last additional waypoint corresponds to
    %"standard" waypoint onto the trajectory
    for i = 1:p-1
        %Waypoint generation
        waypoint = puntoCorr - i * [deltaN deltaE 0];
        waypoint(3) = puntoDes(3);
        matrix(i,:) = waypoint;
    end
    
    M = matrix;   
   
end