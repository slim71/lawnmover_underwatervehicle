function [transectPoints,turnPoints, orInTurn] = matricesWayPoints(vett, longer, shorter, dist, trans, rot, yaw,vel)
%MATRICESWAYPOINTS generates matrices containing position and orientation
% waypoints in the local reference system aligned with the first transect; 
% waypoints are generated for both transects and turns
%
%   -vett: coordinates of the first point of the area of interest; 1x3 [m m m]
%   -longer, shorter: length of the two sides of the area; [m], [m]
%   -dist: distance between two waypoints on the same transect; [m]
%   -trans: distance between two transects; [m]
%   -rot: movement direction indicator, based on the first turn direction; [-1/+1]
%   -yaw: reference yaw angle at which the trajectory is oriented; [°]

    pts = [];
    count = 0; %Travelled transect number
    
    %% Mathematical area management to ensure a complete scan
    %Ensuring the vehicle will use the complete transect length
    if mod(longer,dist)==0
        longerRange = [0:dist:longer];
    else
        longerRange = [[0:dist:longer] [longer:longer]];
    end
    
    %Setting the number of transects to cover
    if mod(shorter,trans) <= trans/2
        shorterRange =[0:trans:shorter];
    else
        shorterRange =[0:trans:shorter-mod(shorter,trans)+trans];
    end

    %% Actual waypoints generation
    for short = shorterRange %For each transect, indicated by "page"
        page = [];
        
        for  long = longerRange %For each waypoint position
            %Waypoint generation
            way = vett' + ned2Traj(yaw)'*[long; rot*short; 0];
            %Adding the waypoint to the row indicating the current contemplated transect
            page = cat(1,page,way');  
        end
        
        %Even numbered transects are travelled "bottom-top",
        %odd numbered transects are travelled "top-bottom"
        if mod(count,2) == 1
            page = flip(page);
        end
        
        count = count+1;
        
        %Adding the row to the entire transects matrix
        pts = cat(3,pts,page);
    end
    
    transectPoints = pts;
    %Contextually calling a function to generate position and orientation
    %of turn waypoints
    [turnPoints, orInTurn] = virataWayPoints();
    
    %% Turn waypoints generation
    function [turnWayPoints, refOr] = virataWayPoints()
    %Nested function to generate position and orientation of turn
    %waypoints using a parameterized circumference

        %Radius of the circumference
        r = trans/2;
        
        %Number of waypoints to generate; actual number varying depending
        %on the circumference radius to achieve better precision
        if trans<4
            nwp = 200;
        else
            nwp = floor(pi/((vel/r)/10));
        end

        %Parameter, normalized for numeric precision
        t = 1/nwp:1/nwp:(1-1/nwp);
        points = [];
        orients = [];
        
        s = size(transectPoints);
        
        %Position references for each turn waypoint
        for n = [1:1:s(3)-1] %For each turn
            page2 = [];%Row containing position of waypoints onto the considered turn
            pageOr = [];%Row containing orientation of waypoints onto the considered turn

            %Position references
            for phi = t %For each portion of circumference swept
                %Waypoint generation
                pt = transectPoints(s(1),:,n)' + ned2Traj(yaw)'*(rot*[0 r 0] + r*[((-1)^(n-1))*sin(pi*phi) cos(pi*phi) 0])';
                page2 = cat(1, page2, pt');%Adding the waypoint to the row
            end
            
            %References are generated in the same direction every time, so
            %a flip is needed to ensure correct position handling
            if rot == 1
                page2 = flip(page2);
            end
            
            %Orientation references
            delta = 180/nwp:180/nwp:(180-180/nwp);%Parameter for angle discretization
            %Actual generation based onto the first turn direction
            if rot == 1 %First turn CCW respect the Down axis
                if mod(n,2)~=0 %Odd turns
                    angles = [zeros(nwp-1,1) zeros(nwp-1,1) rad2deg(wrapToPi(deg2rad(yaw+delta')))];
                else %Even turns
                    angles = [zeros(nwp-1,1) zeros(nwp-1,1) rad2deg(wrapToPi(deg2rad(yaw+180-delta')))];
                end
            else %rot == -1; first turn CW respect the Down axis
                if mod(n,2)~=0 %Odd turns
                    angles = [zeros(nwp-1,1) zeros(nwp-1,1) rad2deg(wrapToPi(deg2rad(yaw-delta')))];
                else %Even turns
                    angles = [zeros(nwp-1,1) zeros(nwp-1,1) rad2deg(wrapToPi(deg2rad(yaw+180+delta')))];
                end
            end
                
            points = cat(3, points, page2);
            orients = cat(3, orients, angles);
        end

        turnWayPoints = points;
        refOr = orients;

    end

end