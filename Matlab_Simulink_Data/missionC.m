disp('Loading Mission parameters...')

%% The tank is defined through its corners (this will not be modified)

cornerA = [43.781381;
           11.282793]; % Lat/Lon [decimal degrees]
       
cornerB = [43.780975;
           11.283505]; % Lat/Lon [decimal degrees]
       
cornerC = [43.780189;
           11.282698]; % Lat/Lon [decimal degrees]
       
cornerD = [43.780602;
           11.281956]; % Lat/Lon [decimal degrees]
         

%% Mission Parameters

initPoint = [ 43.780796;
              11.282739;
               0       ];           % Lat/Lon [decimal degrees], depth [m]
           
initInspectedSide = 2;              % 1 - AB
                                    % 2 - BC
                                    % 3 - CD
                                    % 4 - DA
                                    %
                                    % the first point is at the half of the
                                    % side
                                   
depth = 5;                          % [m]
distanceFromWall = 8;               % [m]

inspectionDirection = 'clockwise';  % it can be 'counterclockwise'

relativeDirection = 30;             % [deg] w.r.t. the normal direction to
                                    % the wall (positive clockwise)
                                    
speed = 0.4;                        % [m/s]

disp('Done')